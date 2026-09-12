import sys

with open("ArduSub/mode.h", "r") as f:
    lines = f.readlines()

start_idx = -1
end_idx = -1
insert_idx = -1

for i, line in enumerate(lines):
    if "#include <AP_CustomEKF/AP_CustomEKF_Distance.h>" in line:
        start_idx = i
    if start_idx != -1 and "uint32_t _last_ekf_update_ms;" in line:
        end_idx = i + 2 # include `};` and newline
        break

for i, line in enumerate(lines):
    if "class ModePoshold : public ModeAlthold" in line:
        # Find end of ModePoshold
        for j in range(i, len(lines)):
            if lines[j].strip() == "};":
                insert_idx = j + 2
                break
        break

print(f"Start: {start_idx}, End: {end_idx}, Insert: {insert_idx}")

if start_idx != -1 and insert_idx != -1:
    extracted = lines[start_idx:end_idx]
    del lines[start_idx:end_idx]
    
    # Recalculate insert index since lines were removed before it
    if insert_idx > start_idx:
        insert_idx -= (end_idx - start_idx)
        
    lines.insert(insert_idx, "".join(extracted) + "\n")
    
    with open("ArduSub/mode.h", "w") as f:
        f.writelines(lines)
    print("Fixed!")
