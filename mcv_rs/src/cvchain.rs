use anyhow::{Result, Context};
use opencv::{
    core::{self, Mat, Size, Point, Scalar, Vector, Vec3f, Vec4i, Point2f},
    imgcodecs,
    imgproc,
    prelude::*,
    highgui,
};
use sensor_msgs::msg::Image;
use crate::cvbridge_rs::CvBridge; // 桥接器
use std::cmp::Ordering;

pub struct CvChain {
    pub mat: Mat, // 原始图像
    pub edges: Mat, // Canny边缘
    pub threshold: Mat, // 二值化
    pub contours: Vector<Vector<Point>>, // Contours轮廓
    pub circles: Option<Vector<Vec3f>>, // 霍夫圆
    pub lines: Option<Vector<Vec4i>>,   // 霍夫线
}

/// 基本功能方法
impl CvChain {
    pub fn new(mat: Mat) -> Self {
        let edges = Mat::default();
        let contours = Vector::<Vector<Point>>::new();
        let threshold = Mat::default();
        let circles = None;
        let lines = None;
        Self { mat, edges, contours, threshold, circles, lines }
    }

    /// 加载图像
    pub fn from_path(path: &str, flags: i32) -> Self {
        let mat = imgcodecs::imread(path, flags)
            .with_context(|| format!("读取图像失败:{}", path))
            .unwrap();
        Self::new(mat)
    }

    // mat展示
    pub fn show(self, window_name: &str) -> Self {
        if let Err(e) = highgui::imshow(window_name, &self.mat) {
            eprintln!("Mat展示失败: {:?}", e);
        }
        highgui::wait_key(0).unwrap();
        self
    }

    /// Canny边缘展示
    pub fn show_edges(self, window_name: &str) -> Self {
        if let Err(e) = highgui::imshow(window_name, &self.edges) {
            eprintln!("Canny边缘展示失败: {:?}", e);
        }
        highgui::wait_key(0).unwrap();
        self
    }

    /// 二值化展示
    pub fn show_threshold(self, window_name: &str) -> Self {
        if let Err(e) = highgui::imshow(window_name, &self.threshold) {
            eprintln!("二值化展示失败: {:?}", e);
        }
        highgui::wait_key(0).unwrap();
        self
    }



    /// 按比例裁剪
    /// 示例：ratio_cut(0.1, 0.1, 0.1, 0.1) 10%裁剪
    pub fn ratio_cut(
        &mut self,
        left_ratio: f32,
        right_ratio: f32,
        top_ratio: f32,
        bottom_ratio: f32,
    ) -> &mut Self {
        let rows = self.mat.rows();
        let cols = self.mat.cols();

        let left = (cols as f32 * left_ratio).round() as i32;
        let right = (cols as f32 * (1.0 - right_ratio)).round() as i32;
        let top = (rows as f32 * top_ratio).round() as i32;
        let bottom = (rows as f32 * (1.0 - bottom_ratio)).round() as i32;

        let roi = core::Rect::new(
            left,
            top,
            right - left,
            bottom - top,
        );

        if let Ok(roi) = opencv::core::Mat::roi(&self.mat, roi) {
            self.mat = roi.try_clone().context("ROI 克隆失败").unwrap();
        } else {
            eprintln!("ROI 截取失败");
            return self;
        }
        self
    }
}

/// 图像处理方法
impl CvChain {
    /// ROS Image 转换为 RGB
    pub fn from_ros_image(msg: &Image) -> Self {
        let mat_rgb = CvBridge::imgmsg_to_cv2(msg)
            .context("ROS图像消息转RGB_MAT失败")
            .unwrap();
        Self::new(mat_rgb)
    }

    /// 泛用转换方法
    /// code: imgproc::COLOR_BGR2RGB，COLOR_BGR2GRAY等
    pub fn cvtColor(&mut self, code: i32) -> &mut Self {
        let mut dst = Mat::default();
        if let Err(e) = imgproc::cvt_color(&self.mat, &mut dst, code, 0) {
            eprintln!("颜色空间转换失败: {}", code);
            return self;
        }
        self.mat = dst;
        self
    }

    /// 高斯滤波
    pub fn blur(&mut self, ksize: Size) -> &mut Self {
        let mut dst = Mat::default();
        if let Err(e) = imgproc::gaussian_blur(&self.mat, &mut dst, ksize, 0.0, 0.0, core::BORDER_DEFAULT) {
            eprintln!("高斯滤波失败: {:?}", e);
            return self;
        }
        self.mat = dst;
        self
    }

    /// 二值化，修改threshold
    /// thresh: 阈值，max_val: 最大值
    /// type: cv::THRESH_BINARY, cv::THRESH_BINARY_INV, cv::THRESH_TRUNC, cv::THRESH_TOZERO, cv::THRESH_TOZERO_INV
    /// 示例：threshold(127.0, 255.0, imgproc::THRESH_BINARY)
    pub fn threshold(&mut self, thresh: f64, max_val: f64, type_: i32) -> &mut Self {
        let mut dst = Mat::default();
        if let Err(e) = imgproc::threshold(&self.mat, &mut dst, thresh, max_val, type_) {
            eprintln!("二值化失败: {:?}", e);
            return self;
        }
        self.threshold = dst;
        self
    }

    /// Canny边缘检测，修改edges
    /// threshold1: 较小的阈值，threshold2: 较大的阈值
    /// 示例：canny(50.0, 150.0)
    pub fn canny(&mut self, threshold1: f64, threshold2: f64) -> &mut Self {
        let mut edges = Mat::default();
        if let Err(e) = imgproc::canny(&self.mat, &mut edges, threshold1, threshold2, 3, false) {
            eprintln!("Canny边缘检测失败: {:?}", e);
            return self;
        }
        self.edges = edges;
        self
    }

    /// 提取轮廓，修改contours
    /// mode: imgproc::RETR_EXTERNAL等
    /// method: imgproc::CHAIN_APPROX_SIMPLE等
    /// 示例：find_contours(imgproc::RETR_EXTERNAL, imgproc::CHAIN_APPROX_SIMPLE)
    pub fn find_contours(
        self,
        mode: i32,
        method: i32,
    ) -> Self {
        let mut contours = Vector::<Vector<Point>>::new();
        if let Err(e) = imgproc::find_contours(
            &self.mat,
            &mut contours,
            mode,
            method,
            core::Point::new(0, 0),
        ) {
            eprintln!("查找轮廓失败: {:?}", e);
            return self;
        }
        self.contours = contours;
        self
    }

    /// 单区间掩膜操作
    /// lower: 下限值，upper: 上限值
    /// 示例黄色掩膜：mask(Scalar::new(0.0, 100.0, 100.0, 0.0), Scalar::new(100.0, 255.0, 255.0, 0.0))
    pub fn mask(&mut self, lower: Scalar, upper: Scalar) -> &mut Self {
        let mut mask = Mat::default();
        core::in_range(&self.mat, &lower, &upper, &mut mask).unwrap();
        let mat_copy = self.mat.clone();
        if let Err(e) = core::bitwise_and(&mat_copy, &mat_copy, &mut self.mat, &mask) {
            eprintln!("单区间掩膜操作失败: {:?}", e);
        }
        self
    }

    /// 双区间掩膜操作
    /// lower1, upper1: 第一区间的下限和上限
    /// lower2, upper2: 第二区间的下限和上限
    /// 示例红色掩膜：double_mask(Scalar::new(0.0, 100.0, 100.0, 0.0), Scalar::new(100.0, 255.0, 255.0, 0.0), Scalar::new(0.0, 0.0, 100.0, 0.0), Scalar::new(100.0, 100.0, 255.0, 0.0))
    pub fn double_mask(
        &mut self,
        lower1: Scalar,
        upper1: Scalar,
        lower2: Scalar,
        upper2: Scalar,
    ) -> &mut Self {
        let mut mask1 = Mat::default();
        let mut mask2 = Mat::default();
        core::in_range(&self.mat, &lower1, &upper1, &mut mask1).unwrap();
        core::in_range(&self.mat, &lower2, &upper2, &mut mask2).unwrap();
        let mut mask1_copy = mask1.clone();
        core::bitwise_or(&mask1, &mask2, &mut mask1_copy, &core::no_array()).unwrap();
        let mat_copy = self.mat.clone();
        if let Err(e) = core::bitwise_and(&mat_copy, &mat_copy, &mut self.mat, &mask1) {
            eprintln!("双区间掩膜操作失败: {:?}", e);
        }
        self
    }
}

/// 霍夫处理方法
impl CvChain {
    /// 霍夫圆
    /// method: imgproc::HOUGH_GRADIENT等
    /// dp: 累加器分辨率
    /// min_dist: 最小圆心距
    /// param1: Canny高阈值
    /// param2: 圆心检测的阈值
    /// min_radius: 最小圆半径
    /// max_radius: 最大圆半径
    /// 示例：hough_circles(imgproc::HOUGH_GRADIENT, 1.0, 20.0, 50.0, 30.0, 0, 0)
    pub fn hough_circles(
        &mut self,
        method: i32,
        dp: f64,
        min_dist: f64,
        param1: f64,
        param2: f64,
        min_radius: i32,
        max_radius: i32,
    ) -> &mut Self {
        let mut circles = Vector::<Vec3f>::new();
        if let Err(e) = imgproc::hough_circles(
            &self.mat,
            &mut circles,
            method,
            dp,
            min_dist,
            param1,
            param2,
            min_radius,
            max_radius,
        ) {
            eprintln!("霍夫圆检测失败: {:?}", e);
            return self;
        }

        self.circles = Some(circles);
        self
    }

    /// 霍夫线
    /// rho: 分辨率
    /// theta: 角分辨率
    /// threshold: 累加器阈值
    /// min_line_length: 最小线段长度
    /// max_line_gap: 最大线段间隔
    /// 示例：hough_lines_p(1.0, 1.0, 100, 50.0, 10.0)
    pub fn hough_lines_p(
        &mut self,
        rho: f64,
        theta: f64,
        threshold: i32,
        min_line_length: f64,
        max_line_gap: f64,
    ) -> &mut Self {
        let mut lines = Vector::<Vec4i>::new();
        if let Err(e) = imgproc::hough_lines_p(
            &self.mat,
            &mut lines,
            rho,
            theta,
            threshold,
            min_line_length,
            max_line_gap,
        ) {
            eprintln!("霍夫线检测失败: {:?}", e);
            return self;
        }

        self.lines = Some(lines);
        self
    }

    /// 获取最长直线，独立方法
    /// 示例：let Some(longest) = chain.get_longest_line();
    pub fn get_longest_line(&self) -> Option<Vec4i> {
        let lines = self.lines.as_ref()?;

        lines.iter().max_by(|a, b| {
            let len_a = {
                let dx = (a[0] - a[2]) as f64;
                let dy = (a[1] - a[3]) as f64;
                (dx * dx + dy * dy).sqrt()
            };
            let len_b = {
                let dx = (b[0] - b[2]) as f64;
                let dy = (b[1] - b[3]) as f64;
                (dx * dx + dy * dy).sqrt()
            };

            len_a.partial_cmp(&len_b).unwrap_or(Ordering::Equal)
        }).map(|v| Vec4i::from(*v))
    }
}

/// 多轮廓处理方法
impl CvChain {
    /// 过滤靠近轮廓（基于质心距离）
    pub fn filter_contours_by_centroid(&mut self, min_dist: f64) -> &mut Self {
        use opencv::{core::Vector, core::norm, core::Point2f, imgproc};

        let mut centers: Vector<Point2f> = Vector::new();
        let mut valid: Vector<Vector<Point>> = Vector::new();

        for cnt in &self.contours {
            let m = imgproc::moments(&cnt, false).unwrap();
            if m.m00 != 0.0 {
                let cx = (m.m10 / m.m00) as f32;
                let cy = (m.m01 / m.m00) as f32;
                let center = Point2f::new(cx, cy);

                // 需要将 `center - *c` 转换为 Point2f，再调用 norm
                let is_far = (0..centers.len()).all(|i| {
                    let c = centers.get(i).unwrap();
                    let dx = center.x - c.x;
                    let dy = center.y - c.y;
                    let dist = (dx * dx + dy * dy).sqrt();
                    dist >= min_dist as f32
                });

                if is_far {
                    centers.push(center);
                    valid.push(cnt.clone());
                }
            }
        }

        self.contours = valid;
        self
    }

    /// 过滤小轮廓（基于面积阈值）
    pub fn filter_contours_by_area(&mut self, min_area: f64) -> &mut Self {
        self.contours = self.contours
            .iter()
            .filter(|cnt| opencv::imgproc::contour_area(cnt, false).unwrap() > min_area)
            .cloned()
            .collect();
        self
    }

    /// 过滤轮廓（基于长宽比）
    pub fn filter_contours_by_aspect_ratio(&mut self, min_ratio: f64, max_ratio: f64) -> &mut Self {
        self.contours = self.contours
            .iter()
            .filter(|cnt| {
                let rect = opencv::imgproc::bounding_rect(cnt).unwrap();
                let aspect_ratio = rect.width as f64 / rect.height as f64;
                aspect_ratio >= min_ratio && aspect_ratio <= max_ratio
            })
            .cloned()
            .collect();
        self
    }
}

/// 单轮廓处理方法
impl CvChain {
    /// 轮廓近似（拉梅-道格拉斯-普克（RDP）算法）
    /// epsilon: 近似精度，closed: 是否闭合
    /// 示例：approx_contours(0.01, true)
    pub fn approx_contours(&mut self, epsilon: f64, closed: bool) -> &mut Self {
        let mut refined: Vec<Vec<Point>> = Vec::new();

        for cnt in &self.contours {
            let mut approx: Vector<Point> = Vector::new();
            if let Err(e) = imgproc::approx_poly_dp(
                &Vector::from(cnt.clone()),
                &mut approx,
                epsilon,
                closed,
            ) {
                eprintln!("轮廓近似失败: {:?}", e);
                continue;
            }

            // 将 Vector<Point> 转回 Vec<Point>
            refined.push(approx.to_vec());
        }

        self.contours = refined;
        self
    }
}

/// 绘制方法
impl CvChain {
    /// 绘制轮廓+展示
    pub fn show_contours(&self) -> &Self {
        let mut out = self.mat.clone();
        for cnt in self.contours.iter() {
            if let Err(e) = imgproc::draw_contours(
                &mut out,
                &Vector::from_iter([cnt.clone()]),
                -1,
                Scalar::new(0.0, 255.0, 0.0, 255.0), // 绿色
                2,
                imgproc::LINE_8,
                &core::no_array(),
                i32::MAX,
                Point::new(0, 0),
            ) {
                eprintln!("绘制轮廓出错: {:?}", e);
            }
        }
        
        if let Err(e) = highgui::imshow("Contours", &out) {
            eprintln!("imshow error: {:?}", e);
        }
        self
    }

    /// 绘制霍夫圆+展示
    pub fn show_circles(&self) -> &Self {
        let mut out = self.mat.clone();
        if let Some(circles) = &self.circles {
            for circle in circles {
                let center = Point::new(circle[0] as i32, circle[1] as i32);
                let radius = circle[2] as i32;
                if let Err(e) = imgproc::circle(&mut out, center, radius, Scalar::new(0.0, 255.0, 0.0, 255.0), 2, imgproc::LINE_AA, 0) {
                    eprintln!("绘制圆出错: {:?}", e);
                }
            }
        }
        if let Err(e) = highgui::imshow("Circles", &out) {
            eprintln!("imshow error: {:?}", e);
        }
        self
    }

    /// 绘制霍夫线+展示
    pub fn show_lines(&self) -> &Self {
        let mut out = self.mat.clone();
        if let Some(lines) = &self.lines {
            for line in lines {
                let p1 = Point::new(line[0], line[1]);
                let p2 = Point::new(line[2], line[3]);
                if let Err(e) = imgproc::line(&mut out, p1, p2, Scalar::new(0.0, 255.0, 0.0, 255.0), 2, imgproc::LINE_AA, 0) {
                    eprintln!("绘制线出错: {:?}", e);
                }
            }
        }
        if let Err(e) = highgui::imshow("Lines", &out) {
            eprintln!("imshow error: {:?}", e);
        }
        self
    }
}