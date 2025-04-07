use std::sync::Mutex;
use crate::target::Target;

pub struct Path {
    current_index: Mutex<usize>,
    waypoints: Mutex<Vec<Target>>,
}

#[allow(dead_code)]
impl Path {
    pub fn new() -> Self {
        Path {
            current_index: Mutex::new(0),
            waypoints: Mutex::new(Vec::new()),
        }
    }

    // 添加航点
    pub fn add_waypoint(&self, waypoint: Target) {
        self.waypoints.lock().unwrap().push(waypoint);
    }

    // 删除航点
    pub fn remove_waypoint(&self, erase_num: usize) -> Result<(), String> {
        let mut waypoints = self.waypoints.lock().unwrap();
        
        if waypoints.is_empty() {
            return Err("path: 路径已经删空！".to_string());
        }

        if erase_num < waypoints.len() {
            waypoints.remove(erase_num);
            Ok(())
        } else {
            Err(format!("path: 非法航点号: {}", erase_num))
        }
    }

    // 获取下一个航点
    pub fn get_next_waypoint(&self) -> Option<Target> {
        let mut current_index = self.current_index.lock().unwrap();
        let waypoints = self.waypoints.lock().unwrap();

        if *current_index < waypoints.len() {
            let waypoint = waypoints[*current_index].clone();
            *current_index += 1;
            Some(waypoint)
        } else {
            *current_index = 0;
            None // 所有航点已发送
        }
    }
}