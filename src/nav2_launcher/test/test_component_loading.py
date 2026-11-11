"""Exercise lost load acknowledgements without starting robot components."""
from concurrent.futures import Future
from types import SimpleNamespace

import pytest
from composition_interfaces.srv import ListNodes, LoadNode

from nav2_launcher.recoverable_components import load_component


class Client:
    def __init__(self, handler):
        self.handler = handler
        self.calls = 0
        self.removed = []

    def wait_for_service(self, timeout_sec):
        return True

    def call_async(self, request):
        self.calls += 1
        response = self.handler(self.calls)
        future = Future()
        if response is not None:
            future.set_result(response)
        return future

    def remove_pending_request(self, future):
        self.removed.append(future)


@pytest.mark.parametrize('failure', ['lost_reply', 'lost_request', 'rejected', 'none'])
def test_loading_recovers_without_duplicates(failure):
    names = []
    request = LoadNode.Request()
    request.node_name, request.node_namespace = 'controller_server', '/robot'
    full_name = '/robot/controller_server'

    def load(attempt):
        if attempt == 1 and failure == 'lost_request':
            return None
        if attempt == 1 and failure == 'rejected':
            return LoadNode.Response(success=False, error_message='temporary load failure')
        assert full_name not in names, 'Attempted to load a duplicate component'
        names.append(full_name)
        if failure == 'lost_reply':
            return None
        return LoadNode.Response(success=True, full_node_name=full_name, unique_id=1)

    loader = Client(load)
    listing = Client(lambda attempt: ListNodes.Response(full_node_names=names.copy(), unique_ids=[1] * len(names)))
    logger = SimpleNamespace(info=lambda text: None, warning=lambda text: None)
    result = load_component(loader, listing, request, logger, lambda: False, timeout=0.01)
    assert result == full_name
    assert names == [full_name]
    assert loader.calls == (2 if failure in ('lost_request', 'rejected') else 1)
    assert len(loader.removed) == (1 if failure in ('lost_reply', 'lost_request') else 0)


def test_persistent_failure_is_bounded_and_shutdown_stops_loading():
    request = LoadNode.Request(node_name='controller_server')
    loader = Client(lambda attempt: LoadNode.Response(success=False, error_message='bad plugin'))
    listing = Client(lambda attempt: ListNodes.Response())
    logger = SimpleNamespace(info=lambda text: None, warning=lambda text: None)
    with pytest.raises(RuntimeError, match='bounded load checks'):
        load_component(loader, listing, request, logger, lambda: False, timeout=0.01)
    assert loader.calls == 3
    assert load_component(loader, listing, request, logger, lambda: True, timeout=0.01) is None
    assert loader.calls == 3


def test_unknown_load_result_does_not_allow_retry_without_listing():
    request = LoadNode.Request(node_name='controller_server')
    loader = Client(lambda attempt: None)
    listing = Client(lambda attempt: ListNodes.Response() if attempt == 1 else None)
    logger = SimpleNamespace(info=lambda text: None, warning=lambda text: None)
    with pytest.raises(RuntimeError):
        load_component(loader, listing, request, logger, lambda: False, timeout=0.01)
    assert loader.calls == 1
