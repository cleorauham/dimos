// NativeModule integration test — pong side.
//
// Receives Twist messages on `data` and echoes each one back on `confirm`.

use dimos_native_module::{LcmTransport, NativeModule};
use lcm_msgs::geometry_msgs::Twist;

#[tokio::main]
async fn main() {
    let transport = LcmTransport::new().await.expect("Failed to create transport");
    let mut module = NativeModule::from_args(transport).await.expect("Failed to create module");

    let mut data = module.input("data", Twist::decode);
    let confirm = module.output("confirm", Twist::encode);
    let _handle = module.spawn();

    eprintln!("pong ready");

    loop {
        match data.recv().await {
            Some(msg) => {
                confirm.publish(&msg).await.ok();
            }
            None => break,
        }
    }
}
