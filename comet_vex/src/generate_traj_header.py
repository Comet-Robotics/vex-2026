import os
import json

TRAJ_DIR = "paths"
OUTPUT_FILE = "include/paths.hpp"

def generate_header():
    with open(OUTPUT_FILE, "w") as out:
        out.write("// AUTO-GENERATED FILE. DO NOT EDIT.\n")
        out.write("#pragma once\n")
        out.write("#include <unordered_map>\n")
        out.write("#include <string>\n\n")
        
        out.write("const std::unordered_map<std::string, std::string> AUTO_TRAJECTORIES = {\n")
        
        for filename in os.listdir(TRAJ_DIR):
            if filename.endswith(".traj"):
                name = filename.replace(".traj", "")
                filepath = os.path.join(TRAJ_DIR, filename)
                
                with open(filepath, "r") as f:
                    # Read the JSON to ensure it's valid, then dump it as a string
                    data = json.load(f)
                    json_str = json.dumps(data, separators=(',', ':')) # remove whitespace for compactness
                    
                    # Write as a C++11 raw string literal inside the map
                    out.write(f'    {{"{name}", R"({json_str})"}},\n')
                    
        out.write("};\n")

if __name__ == "__main__":
    generate_header()