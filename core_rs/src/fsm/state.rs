use async_trait::async_trait;

#[async_trait]
#[allow(dead_code)]
pub trait State: Send + Sync { // 添加trait bound线程约束
    async fn enter(&mut self) {}
    async fn update(&mut self) {}
    async fn exit(&mut self) {}
}
