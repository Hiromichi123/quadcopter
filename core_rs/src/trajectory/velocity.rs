use geometry_msgs::msg::TwistStamped;
use rclrs::Clock;

// 速度
pub struct Velocity {
    twist_stamped: TwistStamped,
}

impl Velocity {
    pub fn default() -> Self {
        Velocity {
            twist_stamped: TwistStamped::default(),
        }
    }
    
    pub fn new(v_x: f64, v_y: f64, v_z: f64, v_yaw: f64, v_pitch: f64, v_roll: f64) -> Self {
        let mut twist = TwistStamped::default();
        twist.header.stamp.sec = (Clock::system().now().nsec / 1_000_000_000) as i32;
        twist.header.stamp.nanosec = (Clock::system().now().nsec % 1_000_000_000) as u32;
        twist.header.frame_id = "base_link".into();
        twist.twist.linear.x = v_x;
        twist.twist.linear.y = v_y;
        twist.twist.linear.z = v_z;
        twist.twist.angular.z = v_yaw;
        twist.twist.angular.y = v_pitch;
        twist.twist.angular.x = v_roll;
        Velocity { twist_stamped: twist }
    }

    // 获取成员
    pub fn get_vx(&self) -> f64 { self.twist_stamped.twist.linear.x }
    pub fn get_vy(&self) -> f64 { self.twist_stamped.twist.linear.y }
    pub fn get_vz(&self) -> f64 { self.twist_stamped.twist.linear.z }
    pub fn get_vyaw(&self) -> f64 { self.twist_stamped.twist.angular.z }
    pub fn get_vpitch(&self) -> f64 { self.twist_stamped.twist.angular.y }
    pub fn get_vroll(&self) -> f64 { self.twist_stamped.twist.angular.x }

    // 获取发布所需的TwistStamped结构体
    pub fn get_twist(&self) -> &TwistStamped { &self.twist_stamped }

    // 设置TwistStamped成员
    pub fn set_vx(&mut self, vx: f64) { self.twist_stamped.twist.linear.x = vx; }
    pub fn set_vy(&mut self, vy: f64) { self.twist_stamped.twist.linear.y = vy; }
    pub fn set_vz(&mut self, vz: f64) { self.twist_stamped.twist.linear.z = vz; }
    pub fn set_vyaw(&mut self, vyaw: f64) { self.twist_stamped.twist.angular.z = vyaw; }
    pub fn set_vpitch(&mut self, vpitch: f64) { self.twist_stamped.twist.angular.y = vpitch; }
    pub fn set_vroll(&mut self, vroll: f64) { self.twist_stamped.twist.angular.x = vroll; }
    // 设置时间戳
    pub fn set_time_now(&mut self) { self.twist_stamped.header.stamp.sec = (Clock::system().now().nsec / 1_000_000_000) as i32;
                                self.twist_stamped.header.stamp.nanosec = (Clock::system().now().nsec % 1_000_000_000) as u32; }
}

// 实现到TwistStamped的转换
impl From<Velocity> for TwistStamped {
    fn from(velocity: Velocity) -> Self {
        velocity.twist_stamped
    }
}

impl From<&Velocity> for TwistStamped {
    fn from(velocity: &Velocity) -> Self {
        velocity.twist_stamped.clone()
    }
}
