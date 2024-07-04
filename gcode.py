motion = ["G0", "G1", "G2", "G3"]
plane = ["G17", "G18", "G19"]
distance = ["G90", "G91"]
feed = ["G93", "G94"]
units = ["G20", "G21"]
tool_length = ["G43", "G49"]
coord_sys = ["G54", "G55", "G56", "G57", "G58", "G59", "G59.1", "G59.2", "G59.3"]
path = ["G61.1"]
non_modal = ["G53"]

all = motion + plane + distance + feed + units + tool_length + coord_sys + path + non_modal
ids = dict()
id = 0

for code in all:
    if code in ids:
        raise f"duplicate code: {code}"
    else:
        ids[code] = id
        id += 1


def decl_enum(name, codes):
    print(f"enum class {name} : std::uint8_t {{")

    for code in codes:
        print(f"    {code.replace(".", "_")} = {ids[code]},")

    print("};\n")


def decl_name(name, codes):
    print(f"inline std::string_view name(const {name} code) {{")
    print("    switch(code) {")

    for code in codes:
        print(f"        case {name}::{code.replace(".", "_")}: return \"{code}\";")
    print("    }")
    print()
    print(f"    PANIC(\"{{}}() invalid code {name}::{{}}\", __func__, std::to_underlying(code));")
    print("}\n")


print("#pragma once")
print()
print("#include <cstdint>")
print("#include <utility>")
print("#include <string_view>")
print()
print("#include \"utils.h\"")
print()

decl_enum("GCode", all)
decl_name("GCode", all)

decl_enum("GCMotion", motion)
decl_name("GCMotion", motion)

decl_enum("GCPlane", plane)
decl_name("GCPlane", plane)

decl_enum("GCDist", distance)
decl_name("GCDist", distance)

decl_enum("GCFeed", feed)
decl_name("GCFeed", feed)

decl_enum("GCUnits", units)
decl_name("GCUnits", units)

decl_enum("GCTLen", tool_length)
decl_name("GCTLen", tool_length)

decl_enum("GCCoord", coord_sys)
decl_name("GCCoord", coord_sys)

decl_enum("GCPath", path)
decl_name("GCPath", path)

decl_enum("GCNonModal", non_modal)
decl_name("GCNonModal", non_modal)
