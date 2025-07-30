// ========================================
// 支持转换的编码格式：
// ========================================
// |   编码格式    |     Mat类型      |   通道数  |    深度   |
// |--------------|------------------|----------|-----------|
// | "mono8"      | CV_8UC1          | 1        | 8-bit     |
// | "mono16"     | CV_16UC1         | 1        | 16-bit    |
// | "bgr8"       | CV_8UC3          | 3        | 8-bit     |
// | "rgb8"       | CV_8UC3          | 3        | 8-bit     |
// | "bgra8"      | CV_8UC4          | 4        | 8-bit     |
// | "rgba8"      | CV_8UC4          | 4        | 8-bit     |
//
// 尚未支持的格式：
// ------------------------------------------
// | "bgr16"      | CV_16UC3         | 3        | 16-bit    |
// | "rgb16"      | CV_16UC3         | 3        | 16-bit    |
// | "32FC1"      | CV_32FC1         | 1        | 32-bit float |
// | "32FC3"      | CV_32FC3         | 3        | 32-bit float |
// | "yuv422"     | -- conversion needed --   |
// | "8UC1"~"8UC4"| CV_8UC1~CV_8UC4  | 1~4      | 8-bit     |
// | "16UC1"~     | CV_16UCx         | x        | 16-bit    |
use opencv::{
    core::{self, Mat, MatTraitConst, CV_8UC1, CV_8UC3, CV_16UC1},
    prelude::*,
};
use sensor_msgs::msg::Image;
use std_msgs::msg::Header;

pub struct CvBridge;

impl CvBridge {
    pub fn imgmsg_to_cv2(msg: &Image) -> opencv::Result<Mat> {
        let rows = msg.height as i32;
        let cols = msg.width as i32;
        let encoding = msg.encoding.as_str();

        let mat_type = match encoding {
            "mono8" => CV_8UC1,
            "bgr8" | "rgb8" => CV_8UC3,
            "mono16" => CV_16UC1,
            _ => {
                return Err(opencv::Error::new(
                    core::StsUnsupportedFormat,
                    format!("Unsupported encoding: {}", encoding),
                ));
            }
        };

        let data = msg.data.clone();
        let mat = Mat::from_slice(&data)?;
        let mat = mat.reshape(
            if mat_type == CV_8UC3 { 3 } else { 1 },
            rows,
        )?;

        Ok(mat.try_clone()?)
    }

    pub fn cv2_to_imgmsg(mat: &Mat, encoding: &str) -> opencv::Result<Image> {
        let size = mat.size()?;
        let width = size.width as u32;
        let height = size.height as u32;

        let data = mat.data_bytes()?.to_vec();

        Ok(Image {
            header: std_msgs::msg::Header::default(), // 可自行设置时间戳/frame_id
            height,
            width,
            encoding: encoding.to_string(),
            is_bigendian: 0,
            step: (mat.step1(0)? as u32),
            data,
        })
    }
}