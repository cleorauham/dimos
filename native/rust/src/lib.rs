pub mod transport;
pub mod lcm;
pub mod module;

pub use transport::Transport;
pub use lcm::LcmTransport;
pub use module::{Input, NativeModule, NativeModuleHandle, Output};

// Re-export LcmOptions so callers don't need to depend on dimos-lcm directly.
pub use dimos_lcm::LcmOptions;
