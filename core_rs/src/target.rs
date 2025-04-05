use std::f64::consts::PI;
use std::time::SystemTime;
use geometry_msgs::msg::PoseStamped;
use rclrs::Clock;

// 目标点
pub struct Target {
    pub reached: bool,
    pose_stamped: PoseStamped,
}

impl Target {
    pub fn new(x: f64, y: f64, z: f64, yaw: f64) -> Self {
        let mut pose = PoseStamped::default();
        pose.header.stamp = Clock::now().into();
        pose.header.frame_id = "base_link".into();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = (yaw / 2.0).sin();
        pose.pose.orientation.w = (yaw / 2.0).cos();
        
        Target {
            reached: false,
            pose_stamped: pose,
        }
    }

    // 获取成员
    pub fn get_x(&self) -> f64 { self.pose_stamped.pose.position.x }
    pub fn get_y(&self) -> f64 { self.pose_stamped.pose.position.y }
    pub fn get_z(&self) -> f64 { self.pose_stamped.pose.position.z }
    pub fn get_yaw(&self) -> f64 {
        let yaw = (2.0 * (self.pose_stamped.pose.orientation.z * self.pose_stamped.pose.orientation.w)).atan2(
            1.0 - 2.0 * (self.pose_stamped.pose.orientation.z * self.pose_stamped.pose.orientation.z));
        if yaw < 0.0 { yaw + 2.0 * PI } else { yaw }
    }
    
    // 获取发布所需的PoseStamped结构体
    pub fn get_pose(&self) -> &PoseStamped { &self.pose_stamped }

    // 设置PoseStamped成员
    pub fn set_x(&mut self, x: f64) { self.pose_stamped.pose.position.x = x; }
    pub fn set_y(&mut self, y: f64) { self.pose_stamped.pose.position.y = y; }
    pub fn set_z(&mut self, z: f64) { self.pose_stamped.pose.position.z = z; }
    pub fn set_yaw(&mut self, yaw: f64) { self.pose_stamped.pose.orientation.z = (yaw / 2.0).sin(); self.pose_stamped.pose.orientation.w = (yaw / 2.0).cos(); }
    // 设置时间戳
    pub fn set_time(&mut self, time: SystemTime) { self.pose_stamped.header.stamp = time.into(); }
}

// 实现到PoseStamped的转换
impl From<Target> for PoseStamped {
    fn from(target: Target) -> Self {
        target.pose_stamped
    }
}

impl From<&Target> for PoseStamped {
    fn from(target: &Target) -> Self {
        target.pose_stamped.clone()
    }
}