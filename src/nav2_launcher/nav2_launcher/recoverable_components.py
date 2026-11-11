"""Bounded, idempotent component loading when a service reply is lost."""
import threading
import time

from composition_interfaces.srv import ListNodes, LoadNode
import launch.logging
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import LoadComposableNodes
from launch_ros.ros_adapters import get_ros_node
from launch_ros.utilities import add_node_name


def service_call(client, request, timeout, stopped):
    deadline = time.monotonic() + timeout
    while not stopped():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        if client.wait_for_service(timeout_sec=min(0.1, remaining)):
            break
    else:
        return None
    future = client.call_async(request)
    ready = threading.Event()
    future.add_done_callback(lambda f: ready.set())
    while not stopped():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        if ready.wait(min(0.1, remaining)):
            return future.result()
    client.remove_pending_request(future)
    future.cancel()
    return None


def load_component(load_client, list_client, request, logger, stopped,
                   timeout=10.0, attempts=3):
    full_name = '/' + '/'.join(part for part in
                               (request.node_namespace.strip('/'), request.node_name) if part)
    # A listing processed after the load request establishes whether it took
    # effect. A missing acknowledgement alone cannot justify a second load.
    for attempt in range(attempts + 1):
        if stopped():
            return None
        listing = service_call(list_client, ListNodes.Request(), timeout, stopped)
        if listing is not None and full_name in listing.full_node_names:
            logger.info(f'Confirmed component {full_name} is loaded')
            return full_name
        if attempt == attempts:
            break
        if listing is None:
            logger.warning(f'Container listing unavailable for {full_name}, check {attempt + 1}/{attempts}')
            continue
        logger.info(f'Loading component {full_name}, attempt {attempt + 1}/{attempts}')
        response = service_call(load_client, request, timeout, stopped)
        if response is not None and response.success:
            logger.info(f'Loaded component {full_name}')
            return full_name
        reason = response.error_message if response is not None else 'load reply timed out'
        logger.warning(f'{full_name}: {reason}. Checking the container before retrying.')
    if stopped():
        return None
    raise RuntimeError(f'Nav2 startup failed after bounded load checks for {full_name}. '
                       'Inspect the component-container log.')


class RecoverableLoadComposableNodes(LoadComposableNodes):
    """Use launch_ros request generation with bounded load reconciliation."""
    def __init__(self, *, target_container, **kwargs):
        self._container_substitutions = normalize_to_list_of_substitutions(target_container)
        self._logger = launch.logging.get_logger('nav2_component_loader')
        super().__init__(target_container=target_container, **kwargs)

    def execute(self, context):
        container = perform_substitutions(context, self._container_substitutions).rstrip('/')
        node = get_ros_node(context)
        self._load_client = node.create_client(LoadNode, container + '/_container/load_node')
        self._list_client = node.create_client(ListNodes, container + '/_container/list_nodes')
        return super().execute(context)

    def _load_node(self, request, context):
        name = load_component(self._load_client, self._list_client, request, self._logger,
                              lambda: context.is_shutdown)
        if name:
            add_node_name(context, name)
