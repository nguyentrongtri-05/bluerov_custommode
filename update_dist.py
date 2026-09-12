with open("ArduSub/mode_poshold_dist.cpp", "r") as f:
    code = f.read()

old_logic = """    // Giữ khoảng cách an toàn (> 50cm)
    if (current_dist_m <= 0.5f && body_rates_cms.x > 0) {
        body_rates_cms.x = 0; // Chặn hoàn toàn tốc độ tới
    } 
    // Giảm tốc độ dần nếu khoảng cách từ 1.0m đến 0.5m
    else if (current_dist_m < 1.0f && body_rates_cms.x > 0) {
        float allowed_ratio = (current_dist_m - 0.5f) / 0.5f; // Từ 0.0 đến 1.0
        float max_speed_cms = g.pilot_speed * allowed_ratio;
        if (body_rates_cms.x > max_speed_cms) {
            body_rates_cms.x = max_speed_cms;
        }
    }"""

new_logic = """    float min_dist_m = sub.g.phds_dist_min.get();
    float max_dist_m = min_dist_m + 0.5f; // Vùng giảm tốc bắt đầu trước 50cm so với mức min

    // Giữ khoảng cách an toàn
    if (current_dist_m <= min_dist_m && body_rates_cms.x > 0) {
        body_rates_cms.x = 0; // Chặn hoàn toàn tốc độ tới
    } 
    // Giảm tốc độ dần trong khoảng 50cm trước khi chạm mức min
    else if (current_dist_m < max_dist_m && body_rates_cms.x > 0) {
        float allowed_ratio = (current_dist_m - min_dist_m) / 0.5f; // Từ 0.0 đến 1.0
        float max_speed_cms = g.pilot_speed * allowed_ratio;
        if (body_rates_cms.x > max_speed_cms) {
            body_rates_cms.x = max_speed_cms;
        }
    }"""

code = code.replace(old_logic, new_logic)

with open("ArduSub/mode_poshold_dist.cpp", "w") as f:
    f.write(code)
print("Updated successfully")
