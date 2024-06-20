stop = ["M0", "M1", "M2", "M30"]
tool_change = ["M6"]
spindle = ["M3", "M4", "M5"]

all = stop + tool_change + spindle
ids = dict()
id = 0

for code in all:
    if code in ids:
        raise f"duplicate code: {code}"
    else:
        ids[code] = id
        id += 1


def decl_enum(name, codes):
    print(f"enum class {name} {{")

    for code in codes:
        print(f"    {code.replace(".", "_")} = {ids[code]},")

    print("};\n")


def decl_name(name, codes):
    print(f"inline const char *name(const {name} code) {{")
    print("    switch(code) {")

    for code in codes:
        print(f"        case {name}::{code.replace(".", "_")}: return \"{code}\";")

    print(f"        default: throw std::runtime_error(std::format(\"{{}}() invalid code {name}::{{}}\", __func__, std::to_underlying(code)));")
    print("    }")
    print("}\n")


print("#ifndef MCODE_GEN_H")
print("#define MCODE_GEN_H\n")

decl_enum("MCode", all)
decl_name("MCode", all)

decl_enum("MCStop", stop)
decl_name("MCStop", stop)

decl_enum("MCToolChange", tool_change)
decl_name("MCToolChange", tool_change)

decl_enum("MCSpindle", spindle)
decl_name("MCSpindle", spindle)

print("#endif")
