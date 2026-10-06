import re

with open("ArduSub/mode_poshold_dist.cpp", "r") as f:
    code = f.read()

old_logic = """    float min_dist_m = sub.g.phds_dist_min.get();
    float max_dist_m = min_dist_m + 0.3f; // Vùng giảm tốc bắt đầu trước 50cm so với mức min

    // Giữ khoảng cách an toàn
    if (current_dist_m <= min_dist_m && body_rates_cms.x > 0) {
        body_rates_cms.x = 0; // Chặn hoàn toàn tốc độ tới
    } 
    // Giảm tốc độ dần trong khoảng 50cm trước khi chạm mức min
    else if (current_dist_m < max_dist_m && body_rates_cms.x > 0) {
        float allowed_ratio = (current_dist_m - min_dist_m) / 0.3f; // Từ 0.0 đến 1.0
        float max_speed_cms = g.pilot_speed * allowed_ratio;
        if (body_rates_cms.x > max_speed_cms) {
            body_rates_cms.x = max_speed_cms;
        }
    }"""

new_logic = """    float min_dist_m = sub.g.phds_dist_min.get();
    float max_dist_m = min_dist_m + 0.3f; // Vùng giảm tốc bắt đầu trước 30cm so với mức min

    // --- Giữ khoảng cách và Tự động lùi ---
    // Chỉ kích hoạt tự lùi nếu cảm biến Ping trả về giá trị hợp lệ (> 10cm)
    // Đề phòng trường hợp Ping bị tuột dây (trả về 0) làm tàu lùi điên cuồng vô tận.
    if (current_dist_m > 0.1f) {
        if (current_dist_m < min_dist_m) {
            // Tàu lún vào vùng cấm -> Ép tàu bơi lùi
            float error_m = min_dist_m - current_dist_m; // Xâm nhập bao nhiêu mét
            float auto_reverse_speed_cms = -error_m * 100.0f; // Kp = 100 (Ví dụ: lún 0.1m -> lùi 10cm/s)
            
            // Giới hạn tốc độ lùi tối đa (Max là 50% tốc độ bay thông thường)
            if (auto_reverse_speed_cms < -g.pilot_speed * 0.5f) {
                auto_reverse_speed_cms = -g.pilot_speed * 0.5f;
            }
            
            // Ghi đè tay ga: Dù phi công đẩy tới hay buông tay, tàu vẫn phải lùi
            if (body_rates_cms.x > auto_reverse_speed_cms) {
                body_rates_cms.x = auto_reverse_speed_cms;
            }
        } 
        // Vùng đệm giảm tốc (từ min_dist đến min_dist+30cm)
        else if (current_dist_m < max_dist_m && body_rates_cms.x > 0) {
            float allowed_ratio = (current_dist_m - min_dist_m) / 0.3f;
            float max_speed_cms = g.pilot_speed * allowed_ratio;
            if (body_rates_cms.x > max_speed_cms) {
                body_rates_cms.x = max_speed_cms;
            }
        }
    } else {
        // Tín hiệu Ping < 10cm (có thể do lỗi cáp, Ping chết hoặc quá sát vách). 
        // Tạm thời chặn tay ga tiến (an toàn tuyệt đối), nhưng cho phép phi công lùi tay.
        if (body_rates_cms.x > 0) {
            body_rates_cms.x = 0;
        }
    }"""

code = code.replace(old_logic, new_logic)

with open("ArduSub/mode_poshold_dist.cpp", "w") as f:
    f.write(code)

print("Auto-reverse patch applied")
