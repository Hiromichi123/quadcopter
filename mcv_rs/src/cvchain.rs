use anyhow::{Result, Context};
use opencv::{
    core::{self, Mat, Size},
    imgcodecs,
    imgproc,
    prelude::*,
};
use sensor_msgs::msg::Image;
use crate::cvbridge_rs::CvBridge; // 桥接器

pub struct CvChain {
    mat: Mat, // 原始图像
    edges: Mat, // Canny边缘
    threshold: Mat, // 二值化
    contours: Vec<Vec<Point>>, // Contours轮廓
}

/// 基本功能方法
impl CvChain {
    pub fn new(mat: Mat) -> Self {
        let edges = Mat::default();
        let contours = Vec::new();
        Self { mat, edges, contours }
    }

    /// 加载图像
    pub fn from_path(path: &str, flags: i32) -> Result<Self> {
        let mat = imgcodecs::imread(path, flags)
            .with_context(|| format!("读取图像失败:{}", path))?;
        Ok(Self::new(mat))
    }

    /// 获取内部Mat引用
    pub fn inner(&self) -> &Mat {
        &self.mat
    }

    // 图像展示方法
    pub fn show(self, window_name: &str) -> Result<Self> {
        highgui::imshow(window_name, &self.mat)?;
        highgui::wait_key(0)?;
        Ok(self)
    }
}

/// 图像处理方法
impl CvChain {
    /// ROS Image 转换为 RGB
    pub fn from_ros_image(msg: &Image) -> Result<Self> {
        let mat_rgb = CvBridge::imgmsg_to_cv2(msg)
            .context("ROS图像消息转RGB_MAT失败")?;
        Ok(Self::new(mat_rgb))
    }

    /// 泛用转换方法
    /// code: imgproc::COLOR_BGR2RGB，COLOR_BGR2GRAY等
    pub fn cvtColor(mut self, code: i32) -> Result<Self> {
        let mut dst = Mat::default();
        imgproc::cvt_color(&self.mat, &mut dst, code, 0)
            .with_context(|| format!("颜色空间转换失败: {}", code))?;
        self.mat = dst;
        Ok(self)
    }

    /// 高斯滤波
    pub fn blur(mut self, ksize: Size) -> Result<Self> {
        let mut dst = Mat::default();
        imgproc::gaussian_blur(&self.mat, &mut dst, ksize, 0.0, 0.0, core::BORDER_DEFAULT)
            .context("高斯滤波失败")?;
        self.mat = dst;
        Ok(self)
    }

    /// 二值化，返回threshold
    pub fn threshold(mut self, threshold: Mat, thresh: f64, max_val: f64, type_: i32) -> Result<Self> {
        let mut dst = Mat::default();
        imgproc::threshold(&self.mat, &mut dst, thresh, max_val, type_)
            .context("二值化失败")?;
        self.mat = dst;
        Ok(self)
    }

    /// Canny边缘检测，返回edges
    pub fn canny(mut self, mut edges: Mat, threshold1: f64, threshold2: f64) -> Result<Self> {
        imgproc::canny(&self.mat, &mut edges, threshold1, threshold2, 3, false)
            .context("Canny边缘检测失败")?;
        self.mat = edges;
        Ok(self)
    }

    /// 提取轮廓，返回contours
    pub fn find_contours(
        &self,
        mode: i32,
        method: i32,
    ) -> Result<Vec<Vec<Point>>> {
        let mut contours = opencv::types::VectorOfVectorOfPoint::new();
        imgproc::find_contours(
            &self.mat,
            &mut contours,
            mode,
            method,
            core::Point::new(0, 0),
        )?;
        Ok(contours.to_vec())
    }

    pub fn mask(mut self, lower: Scalar, upper: Scalar) -> Result<Self> {
        let mut mask = Mat::default();
        core::in_range(&self.mat, &lower, &upper, &mut mask)?;
        core::bitwise_and(&self.mat, &self.mat, &mut self.mat, &mask)?;
        Ok(self)
    }

    impl CvChain {
    pub fn double_mask(
        mut self,
        lower1: Scalar,
        upper1: Scalar,
        lower2: Scalar,
        upper2: Scalar,
    ) -> Result<Self> {
        let mut mask1 = Mat::default();
        let mut mask2 = Mat::default();
        core::in_range(&self.mat, &lower1, &upper1, &mut mask1)?;
        core::in_range(&self.mat, &lower2, &upper2, &mut mask2)?;
        core::bitwise_or(&mask1, &mask2, &mut mask1, &core::no_array()?)?;
        core::bitwise_and(&self.mat, &self.mat, &mut self.mat, &mask1)?;
        Ok(self)
    }
}

}
