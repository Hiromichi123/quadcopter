use std::sync::Arc;
use std::time::{Duration, SystemTime};
use rosrust::{Rate, Clock};
use rosrust::log::{info, warn};
use crate::{quadcopter::Quadcopter, target::Target, velocity::Velocity, path::Path};

const PI: f32 = 3.14;
const DEFAULT_POS_CHECK_DISTANCE: f32 = 0.25;

pub struct FlightController {
    quad_node: Arc<Quadcopter>,
    rate: Rate,
}

impl FlightController {
    pub fn new(quad_node: Arc<Quadcopter>) -> Self {
        FlightController {
            quad_node,
            rate: Rate::new(20.0),
        }
    }

    // target定点移动
    pub fn fly_to_target(&mut self, target: &mut Target) {
        while rosrust::is_ok() && !self.pos_check(target) {
            target.set_time(Clock::now().unwrap().into());
            self.quad_node.pos_pub.send(target.get_pose().clone()).unwrap();
            self.rate.sleep();
        }
    }

    // 自身位置检查，distance为默认误差0.25
    pub fn pos_check(&self, target: &mut Target) -> bool {
        if target.reached { return true; }
        let dx = self.quad_node.lidar_pos.x - target.get_x();
        let dy = self.quad_node.lidar_pos.y - target.get_y();
        let dz = self.quad_node.lidar_pos.z - target.get_z();
        let dyaw = (self.quad_node.lidar_pos.yaw - target.get_yaw()).abs();
        target.reached = (dx.powi(2) + dy.powi(2) + dz.powi(2)).sqrt() < DEFAULT_POS_CHECK_DISTANCE && dyaw < 0.1;
        target.reached
    }

    // 严格位置检查
    pub fn pos_check_strict(&self, target: &mut Target, distance_x: f32, distance_y: f32, distance_z: f32) -> bool {
        if target.reached { return true; }
        let dx = (self.quad_node.lidar_pos.x - target.get_x()).abs();
        let dy = (self.quad_node.lidar_pos.y - target.get_y()).abs();
        let dz = (self.quad_node.lidar_pos.z - target.get_z()).abs();
        let dyaw = (self.quad_node.lidar_pos.yaw - target.get_yaw()).abs();
        target.reached = dx < distance_x && dy < distance_y && dz < distance_z && dyaw < 0.1;
        target.reached
    }

    // velocity速度飞行，单次发布
    pub fn fly_by_velocity(&mut self, velocity: &mut Velocity) {
        velocity.set_time(Clock::now().unwrap().into());
        self.quad_node.vel_pub.send(velocity.get_twist().clone()).unwrap();
    }

    // velocity速度飞行，持续时间duration
    pub fn fly_by_vel_duration(&mut self, velocity: &mut Velocity, duration: f32) {
        let start_time = Clock::now().unwrap();
        let start_altitude = self.quad_node.lidar_pos.z;

        while (Clock::now().unwrap() - start_time).as_secs_f32() < duration {
            // 修改z轴反馈
            if (start_altitude - self.quad_node.lidar_pos.z).abs() > 0.1 { velocity.set_vz(start_altitude - self.quad_node.lidar_pos.z); }
            self.fly_by_velocity(velocity);
            self.quad_node.rate.sleep();
        }
    }
}