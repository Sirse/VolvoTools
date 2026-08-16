#!/usr/bin/env python3
"""Generate CommonData.cpp from common/data.yaml.

CommonData::commonConfiguration used to be a hand-maintained hex blob of the vehicle
configuration, which let data.yaml drift out of sync with what the runtime actually saw.
This script makes data.yaml the single source of truth: CMake regenerates CommonData.cpp
from data.yaml on every build where data.yaml (or the script) is newer. The generated array
is byte-for-byte the YAML file as checked out (LF on this repo); the old blob happened to be
CRLF, so no line-ending normalization is performed here - what you read is what you get.
"""

import argparse
import os
import sys


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", help="path to common/data.yaml")
    parser.add_argument("output", help="path to generated CommonData.cpp")
    args = parser.parse_args()

    with open(args.input, "rb") as f:
        data = f.read()

    output_dir = os.path.dirname(os.path.abspath(args.output))
    os.makedirs(output_dir, exist_ok=True)

    lines = []
    lines.append("#include <common/CommonData.hpp>")
    lines.append("")
    lines.append("namespace common {")
    lines.append("")
    lines.append("    // Generated from common/data.yaml by generate_common_data.py.")
    lines.append("    // Do not edit by hand: the vehicle configuration must have a single source of truth.")
    lines.append("    const std::string CommonData::commonConfiguration = {")
    for i in range(0, len(data), 8):
        chunk = data[i:i + 8]
        bytes_hex = ", ".join("0x%02x" % b for b in chunk)
        lines.append("        " + bytes_hex + ",")
    lines.append("    };")
    lines.append("")
    lines.append("} // namespace common")
    lines.append("")

    with open(args.output, "w", newline="\n") as f:
        f.write("\n".join(lines))

    return 0


if __name__ == "__main__":
    sys.exit(main())
