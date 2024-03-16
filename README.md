# nrf-dfu-target

An implementation of the Nordic nRF DFU protocol in Rust that can be used in a `no_std` environment. It is based on the nRF SDK 17 DFU protocol, but is compatible with older versions as well.

The intention is that any transport (BLE, UART, USB) can use this crate for decoding the request, processing the request and encoding the response for the protocol.

You can use any flash device that implements the embedded-storage traits for sync or embedded-storage-async traits for async version as the target. Use feature `async` to enable async version.

It does not support updating more than one firmware type for each `DfuTarget` instance at the moment. 

## Example

For an example usage based on BLE, see the [pinetime-embassy](https://github.com/lulf/pinetime-embassy) firmware which provides the GATT services and event dispatching.
