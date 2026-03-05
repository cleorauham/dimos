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

from concurrent.futures import ThreadPoolExecutor
from typing import Any, Protocol, runtime_checkable

from dimos.core.global_config import GlobalConfig
from dimos.core.module import ModuleBase
from dimos.core.rpc_client import RPCClient
from dimos.core.worker import WorkerPython
from dimos.core.docker_runner import is_docker_module
from dimos.core.worker_docker import WorkerDocker
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@runtime_checkable
class Worker(Protocol):
    """Common interface for all worker types (process-based and Docker)."""

    @property
    def module_count(self) -> int: ...
    def reserve_slot(self) -> None: ...
    def deploy_module(
        self, module_class: type[ModuleBase], global_config: GlobalConfig, kwargs: dict[str, Any] | None = None
    ) -> RPCClient: ...
    def shutdown(self) -> None: ...


class WorkerManager:
    def __init__(self, n_workers: int = 2) -> None:
        self._n_workers = n_workers
        self._workers: list[WorkerPython] = []
        self._docker_workers: list[WorkerDocker] = []
        self._closed = False
        self._started = False

    def start(self) -> None:
        if self._started:
            return
        self._started = True
        for _ in range(self._n_workers):
            worker = WorkerPython()
            worker.start_process()
            self._workers.append(worker)
        logger.info("Worker pool started.", n_workers=self._n_workers)

    def _select_worker(self, module_class: type[ModuleBase]) -> Worker:
        """Pick the right worker for a module: docker worker or least-loaded process worker."""
        if is_docker_module(module_class):
            docker_worker = WorkerDocker()
            self._docker_workers.append(docker_worker)
            return docker_worker

        # Auto-start process workers on first regular module
        if not self._started:
            self.start()

        return min(self._workers, key=lambda w: w.module_count)

    def deploy(
        self, module_class: type[ModuleBase], global_config: GlobalConfig, kwargs: dict[str, Any]
    ) -> RPCClient:
        if self._closed:
            raise RuntimeError("WorkerManager is closed")

        worker = self._select_worker(module_class)
        return worker.deploy_module(module_class, global_config, kwargs=kwargs)

    def deploy_parallel(
        self, module_specs: list[tuple[type[ModuleBase], GlobalConfig, dict[str, Any]]]
    ) -> list[RPCClient]:
        if self._closed:
            raise RuntimeError("WorkerManager is closed")

        # Assign each module to a worker, reserving slots for load balancing
        assignments: list[tuple[Worker, type[ModuleBase], GlobalConfig, dict[str, Any]]] = []
        for module_class, global_config, kwargs in module_specs:
            worker = self._select_worker(module_class)
            worker.reserve_slot()
            assignments.append((worker, module_class, global_config, kwargs))

        # Deploy all modules concurrently
        def _deploy(
            item: tuple[Worker, type[ModuleBase], GlobalConfig, dict[str, Any]],
        ) -> RPCClient:
            worker, module_class, global_config, kwargs = item
            return worker.deploy_module(module_class, global_config, kwargs=kwargs)

        with ThreadPoolExecutor(max_workers=max(len(assignments), 1)) as pool:
            results = list(pool.map(_deploy, assignments))

        return results

    @property
    def workers(self) -> list[WorkerPython]:
        return list(self._workers)

    def close_all(self) -> None:
        if self._closed:
            return
        self._closed = True

        logger.info("Shutting down all workers...")

        for worker in reversed(self._docker_workers):
            try:
                worker.shutdown()
            except Exception as e:
                logger.error(f"Error shutting down docker worker: {e}", exc_info=True)
        self._docker_workers.clear()

        for worker in reversed(self._workers):
            try:
                worker.shutdown()
            except Exception as e:
                logger.error(f"Error shutting down worker: {e}", exc_info=True)
        self._workers.clear()

        logger.info("All workers shut down")
