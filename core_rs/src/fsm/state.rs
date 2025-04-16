use super::state_machine::StateMachine;

#[allow(dead_code)]
pub trait State {
    fn on_enter(&mut self,  _sm: &mut StateMachine) {}
    fn on_exit(&mut self,  _sm: &mut StateMachine) {}
    fn update(&mut self, _sm: &mut StateMachine);
}

#[derive(Debug)]
pub struct TaskState;
impl State for TaskState {
    fn on_enter(&mut self, _sm: &mut StateMachine) {
        println!("Entering state: {:?}", self);
    }

    fn on_exit(&mut self, _sm: &mut StateMachine) {
        println!("Exiting state: {:?}", self);
    }

    fn update(&mut self, _sm: &mut StateMachine) {
        println!("Updating state: {:?}", self);
        // 在这里添加状态更新逻辑
    }
}