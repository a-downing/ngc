#include "physical/SerialTransport.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <limits>
#include <utility>

#ifdef __linux__
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace ngc::physical {
    namespace {
#ifdef __linux__
        std::expected<speed_t, std::string> serialSpeed(
            const std::uint32_t baud) {
            switch (baud) {
                case 110: return B110;
                case 300: return B300;
                case 600: return B600;
                case 1200: return B1200;
                case 2400: return B2400;
                case 4800: return B4800;
                case 9600: return B9600;
                case 19200: return B19200;
                case 38400: return B38400;
                case 57600: return B57600;
                case 115200: return B115200;
                default:
                    return std::unexpected(
                        "unsupported serial baud rate");
            }
        }

        class PosixSerialTransport final : public SerialTransport {
        public:
            static std::expected<
                std::unique_ptr<SerialTransport>, std::string>
            open(
                const HuanyangSpindleConfiguration &configuration,
                const std::chrono::milliseconds timeout) {
                const auto speed = serialSpeed(configuration.baud);
                if (!speed) {
                    return std::unexpected(speed.error());
                }
                const auto descriptor = ::open(
                    configuration.device.c_str(),
                    O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
                if (descriptor < 0) {
                    return std::unexpected(
                        "failed to open spindle serial device "
                        + configuration.device + ": "
                        + std::strerror(errno));
                }

                auto original = termios{};
                if (tcgetattr(descriptor, &original) != 0) {
                    const auto error = std::string(std::strerror(errno));
                    close(descriptor);

                    return std::unexpected(
                        "failed to read spindle serial settings: "
                        + error);
                }
                auto configured = original;
                cfmakeraw(&configured);
                configured.c_cflag |= CLOCAL | CREAD;
                configured.c_cflag &= ~CSIZE;
                switch (configuration.dataBits) {
                    case 5: configured.c_cflag |= CS5; break;
                    case 6: configured.c_cflag |= CS6; break;
                    case 7: configured.c_cflag |= CS7; break;
                    case 8: configured.c_cflag |= CS8; break;
                    default:
                        close(descriptor);

                        return std::unexpected(
                            "unsupported serial data-bit count");
                }
                if (configuration.stopBits == 2) {
                    configured.c_cflag |= CSTOPB;
                } else {
                    configured.c_cflag &= ~CSTOPB;
                }
                if (configuration.parity == SerialParity::None) {
                    configured.c_cflag &= ~(PARENB | PARODD);
                    configured.c_iflag &= ~INPCK;
                } else {
                    configured.c_cflag |= PARENB;
                    configured.c_iflag |= INPCK;
                    if (configuration.parity == SerialParity::Odd) {
                        configured.c_cflag |= PARODD;
                    } else {
                        configured.c_cflag &= ~PARODD;
                    }
                }
                configured.c_cc[VMIN] = 0;
                configured.c_cc[VTIME] = 0;
                if (cfsetispeed(&configured, *speed) != 0
                    || cfsetospeed(&configured, *speed) != 0
                    || tcsetattr(
                        descriptor, TCSANOW, &configured) != 0) {
                    const auto error = std::string(std::strerror(errno));
                    close(descriptor);

                    return std::unexpected(
                        "failed to configure spindle serial device: "
                        + error);
                }

                return std::unique_ptr<SerialTransport>(
                    new PosixSerialTransport(
                        descriptor, original, timeout));
            }

            ~PosixSerialTransport() override {
                static_cast<void>(tcsetattr(
                    m_descriptor, TCSANOW, &m_original));
                close(m_descriptor);
            }

            bool exchange(
                const std::span<const std::uint8_t> request,
                const std::span<std::uint8_t> response,
                std::size_t &responseSize) noexcept override {
                responseSize = 0;
                if (request.empty() || response.size() < 5) {
                    return false;
                }
                if (tcflush(m_descriptor, TCIFLUSH) != 0) {
                    return false;
                }

                const auto deadline =
                    std::chrono::steady_clock::now() + m_timeout;
                auto written = std::size_t{0};
                while (written < request.size()) {
                    if (!wait(POLLOUT, deadline)) {
                        return false;
                    }
                    const auto result = write(
                        m_descriptor, request.data() + written,
                        request.size() - written);
                    if (result > 0) {
                        written += static_cast<std::size_t>(result);
                    } else if (result < 0
                               && errno != EINTR
                               && errno != EAGAIN) {
                        return false;
                    }
                }

                auto expected = std::size_t{0};
                while (expected == 0 || responseSize < expected) {
                    if (!wait(POLLIN, deadline)) {
                        return false;
                    }
                    const auto result = read(
                        m_descriptor,
                        response.data() + responseSize,
                        response.size() - responseSize);
                    if (result > 0) {
                        responseSize +=
                            static_cast<std::size_t>(result);
                    } else if (result < 0
                               && errno != EINTR
                               && errno != EAGAIN) {
                        return false;
                    }
                    if (responseSize >= 3) {
                        expected =
                            static_cast<std::size_t>(response[2]) + 5;
                        if (expected > response.size()
                            || responseSize > expected) {
                            return false;
                        }
                    }
                    if (responseSize == response.size()
                        && expected == 0) {
                        return false;
                    }
                }

                return true;
            }

        private:
            PosixSerialTransport(
                const int descriptor,
                const termios &original,
                const std::chrono::milliseconds timeout)
                : m_descriptor(descriptor),
                  m_original(original),
                  m_timeout(timeout) { }

            bool wait(
                const short events,
                const std::chrono::steady_clock::time_point
                    deadline) noexcept {
                for (;;) {
                    const auto remaining =
                        deadline - std::chrono::steady_clock::now();
                    if (remaining <=
                        std::chrono::steady_clock::duration::zero()) {
                        return false;
                    }
                    const auto milliseconds =
                        std::chrono::duration_cast<
                            std::chrono::milliseconds>(remaining);
                    const auto timeout = static_cast<int>(std::min(
                        milliseconds.count() + 1,
                        static_cast<
                            decltype(milliseconds.count())>(
                            std::numeric_limits<int>::max())));
                    auto descriptor = pollfd{
                        .fd = m_descriptor,
                        .events = events,
                        .revents = 0,
                    };
                    const auto result = poll(&descriptor, 1, timeout);
                    if (result > 0) {
                        return (descriptor.revents & events) != 0
                            && (descriptor.revents
                                & (POLLERR | POLLHUP | POLLNVAL)) == 0;
                    }
                    if (result == 0) {
                        return false;
                    }
                    if (errno != EINTR) {
                        return false;
                    }
                }
            }

            int m_descriptor;
            termios m_original;
            std::chrono::milliseconds m_timeout;
        };
#endif
    }

    std::expected<std::unique_ptr<SerialTransport>, std::string>
    openSerialTransport(
        const HuanyangSpindleConfiguration &configuration,
        const std::chrono::milliseconds timeout) {
        if (timeout <= std::chrono::milliseconds::zero()) {
            return std::unexpected(
                "spindle serial timeout must be positive");
        }
#ifdef __linux__
        return PosixSerialTransport::open(configuration, timeout);
#else
        static_cast<void>(configuration);

        return std::unexpected(
            "Huanyang spindle serial transport requires Linux");
#endif
    }
}
