# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

class Module:
    _module_refs = {} # this value will be set by build. The value is JSONable
    ...
    def __init__(self, ...):
        ...
        for mod_ref_name, rpc_map in self._module_refs.items():
            setattr(self, mod_ref_name, ModuleRefClient(rpc_map, self.config.rpc_pub_sub))
        ...
    ...


# Looks like a module, but all the methods just get send over some transport
# NOTE: arguments should be JSON serializable
class ModuleRefClient:
    def __init__(self, rpc_map, rpc_pub_sub_config):
        for each_key, (rpc_address, rpc_transport_kind) in rpc_map.items():
            setattr(self, each_key, create_rpc_caller(rpc_address, rpc_transport_kind, rpc_pub_sub_config))

import time
import cbor
import threading
import traceback
from typing import Literal
from random import random

from dimos import constants
from dimos.utils.generic import short_id
from dimos.protocol.pubsub.shmpubsub import SharedMemoryPubSubBase
from dimos.protocol.pubsub.lcmpubsub import LCMPubSubBase

init_lock = threading.Lock()
lcm = None
shm = None

def listen_for_rpc_call(rpc_address: str, rpc_callback, rpc_transport_kind: Literal["shm","lcm"], config: dict = {}):
    assert rpc_transport_kind in ["shm","lcm"]
    _call_thread_pool_lock = threading.RLock()
    def callback(raw_bytes: bytes):
        id, time, args, kwargs  = cbor.loads(raw_bytes)
        result = None
        err = None
        try:
            with _call_thread_pool_lock:
                result = rpc_callback(*args, **kwargs)
        except Exception as error:
            # Get the full traceback as a string
            return dict(
                type_name=type(error).__name__,
                type_module=type(error).__module__,
                args=error.args,
                traceback="".join(traceback.format_exception(type(error), error, error.__traceback__)),
            )
        return cbor.dumps((id, time.time(), result, err))

    # transform address for lcm topic
    address_receiving_args = f"/rpc/{short_id(rpc_address)}/req"
    address_receiving_result = f"/rpc/{short_id(rpc_address)}/res"
    def setup_listener(): ...
    def unsub(): ...
    if rpc_transport_kind == "lcm":
        def setup_listener():
            global lcm
            nonlocal unsub
            with init_lock:
                lcm = lcm or LCMPubSubBase(**config)
            def receive_and_respond(data):
                response = callback(data)
                lcm.publish(address_receiving_result, response)
            unsub = lcm.subscribe(address_receiving_args, receive_and_respond)

    if rpc_transport_kind == "shm":
        def setup_listener():
            global shm
            nonlocal unsub
            with init_lock:
                shm = shm or SharedMemoryPubSubBase(**config)

            def receive_and_respond(data):
                response = callback(data)
                lcm.publish(address_receiving_result, response)
            unsub = shm.subscribe(address_receiving_args, receive_and_respond)

    threading.Thread(target=setup_listener, daemon=True).start()
    return unsub

rpc_response_locks = {}
response_locker_lock = threading.Lock()
THREAD_WAIT_TIME = 0.1 # TODO: move to constants
def create_rpc_caller(rpc_address: str, rpc_transport_kind: Literal["shm","lcm"], config: dict = {}):
    assert rpc_transport_kind in ["shm","lcm"]
    address_receiving_args = f"/rpc/{short_id(rpc_address)}/req"
    address_receiving_result = f"/rpc/{short_id(rpc_address)}/res"
    responses = {}
    with response_locker_lock:
        if rpc_address not in rpc_response_locks:
            rpc_response_locks[rpc_address] = threading.Lock()
        response_lock = rpc_response_locks[rpc_address]
    rpc_response_locks.setdefault(rpc_response_locks, response_lock)
    def publish(topic, message_bytes): ...
    def unsub(topic, message_bytes): ...
    def listen(response):
        call_id, time, args, kwargs = cbor.loads(response)
        responses[call_id] = (call_id, time, args, kwargs)
    if rpc_transport_kind == "lcm":
        global lcm
        with init_lock:
            lcm = lcm or LCMPubSubBase(**config)
        unsub = lcm.subscribe(address_receiving_result, listen)
        publish = lcm.publish
    if rpc_transport_kind == "shm":
        global shm
        with init_lock:
            shm = shm or LCMPubSubBase(**config)
        unsub = shm.subscribe(address_receiving_result, listen)
        publish = shm.publish

    def rpc_caller(*args, **kwargs):
        call_id = random()
        serialized_args = cbor.dumps((call_id, time.time(), args, kwargs))
        publish(address_receiving_args, serialized_args)
        output = None
        def grab_response():
            nonlocal output
            while True:
                with response_lock:
                    if call_id in responses:
                        output = responses.pop(call_id)
                        break
                time.sleep(THREAD_WAIT_TIME)

        thread = threading.Thread(target=grab_response)
        thread.start()
        thread.join() # TODO: could add timeout here
        return output

    # TODO: add a method on PubSub that allows for "shutdown if there are no subscribers"
    return rpc_caller, unsub
