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

from __future__ import annotations

from typing import Any

from dimos.core.docker_runner import DockerModule, is_docker_module
from dimos.core.global_config import GlobalConfig
from dimos.core.module import ModuleBase
from dimos.core.rpc_client import RPCClient, RpcCall
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class DockerRPCClient(RPCClient):
    """RPCClient wrapper around a DockerModule.

    Reuses the DockerModule's LCMRPC instance instead of creating a new one,
    and falls back to ``getattr`` (not ``__getattr__``) so that regular methods
    like ``set_transport`` and ``get_rpc_method_names`` are found.
    """

    def __init__(self, docker_module: DockerModule, module_class: type) -> None:
        # Intentionally skip super().__init__ — we reuse DockerModule's RPC.
        self.rpc = docker_module.rpc
        self.actor_class = module_class
        self.remote_name = module_class.__name__
        self.actor_instance = docker_module
        self.rpcs = module_class.rpcs.keys()  # type: ignore[attr-defined]
        self._unsub_fns = docker_module._unsub_fns

    def stop_rpc_client(self) -> None:
        # DockerModule manages its own RPC lifecycle via stop().
        pass

    def __getattr__(self, name: str) -> Any:
        if name in {
            "__class__", "__init__", "__dict__", "__getattr__",
            "rpcs", "remote_name", "remote_instance", "actor_instance",
        }:
            raise AttributeError(f"{name} is not found.")

        if name in self.rpcs:
            original_method = getattr(self.actor_class, name, None)
            return RpcCall(
                original_method, self.rpc, name, self.remote_name,
                self._unsub_fns, self.stop_rpc_client,
            )

        # Use getattr (not __getattr__) so regular DockerModule methods
        # like set_transport, get_rpc_method_names, etc. are found.
        return getattr(self.actor_instance, name)


class WorkerDocker:
    """Manages modules deployed in Docker containers."""

    def __init__(self) -> None:
        self._docker_modules: list[DockerModule] = []

    @property
    def module_count(self) -> int:
        return len(self._docker_modules)

    def reserve_slot(self) -> None:
        """No-op: Docker containers are independent and don't share slots."""

    def deploy_module(
        self, module_class: type[ModuleBase], global_config: GlobalConfig, kwargs: dict[str, Any] | None = None
    ) -> DockerRPCClient:
        kwargs = kwargs or {}
        dm = DockerModule(module_class, **kwargs)
        dm.start()
        self._docker_modules.append(dm)
        return DockerRPCClient(dm, module_class)

    def shutdown(self) -> None:
        for dm in reversed(self._docker_modules):
            try:
                dm.stop()
            except Exception as e:
                logger.error(f"Error stopping Docker module: {e}", exc_info=True)
        self._docker_modules.clear()
