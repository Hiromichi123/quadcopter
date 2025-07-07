use anyhow::{Result, Context};
use opencv::{
    core::{self, Mat, Size},
    imgcodecs,
    imgproc,
    prelude::*,
};

pub struct CvChain {
    mat: Mat,
}

impl CvChain {
    pub fn new(mat: Mat) -> Self {
        Self { mat }
    }

    pub fn from_path(path: &str, flags: i32) -> Result<Self> {
        let mat = imgcodecs::imread(path, flags)
            .with_context(|| format!("Failed to read image from path: {}", path))?;
        Ok(Self::new(mat))
    }

    pub fn to_gray(mut self) -> Result<Self> {
        let mut gray = Mat::default();
        imgproc::cvt_color(&self.mat, &mut gray, imgproc::COLOR_BGR2GRAY, 0)
            .context("Failed to convert to grayscale")?;
        self.mat = gray;
        Ok(self)
    }

    pub fn blur(mut self, ksize: Size) -> Result<Self> {
        let mut dst = Mat::default();
        imgproc::gaussian_blur(&self.mat, &mut dst, ksize, 0.0, 0.0, core::BORDER_DEFAULT)
            .context("Failed to apply Gaussian blur")?;
        self.mat = dst;
        Ok(self)
    }

    pub fn canny(mut self, threshold1: f64, threshold2: f64) -> Result<Self> {
        let mut edges = Mat::default();
        imgproc::canny(&self.mat, &mut edges, threshold1, threshold2, 3, false)
            .context("Failed to apply Canny edge detection")?;
        self.mat = edges;
        Ok(self)
    }

    pub fn into_inner(self) -> Mat {
        self.mat
    }
}
