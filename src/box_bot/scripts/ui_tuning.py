"""Asynchronous tuning with readback and rollback across Nav2 parameter servers."""
import copy
from pathlib import Path
import time

from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.parameter import Parameter, parameter_value_to_python

from tuning_settings import (SETTINGS, PARAMETERS, displayed_values, matches,
                             parameter_values, read_config, save_config)


class NavigationTuning:
    def __init__(self, node, can_apply, config_path):
        self.node = node
        self.can_apply = can_apply
        self.path = Path(config_path).resolve()
        self.clients = {name: (
            node.create_client(GetParameters, '/'+name+'/get_parameters'),
            node.create_client(SetParametersAtomically, '/'+name+'/set_parameters_atomically'))
            for name in PARAMETERS}
        self.actual = {}
        self.values = {}
        self.saved = {}
        self.config_error = ''
        self.config_modified = None
        self.error = ''
        self.read_error = False
        self.message = 'Waiting for live Nav2 tuning parameters.'
        self.operation = None
        self.pending = None
        self.last_read = 0.
        self.updated = 0.
        self.block_navigation = False
        self.load_saved()
        if self.saved:
            node.recovery.speed = self.saved['values']['cruise_speed']

    @property
    def busy(self):
        return self.pending is not None or (self.operation is not None and self.operation['phase'] != 'poll')

    def load_saved(self):
        try:
            self.saved = read_config(self.path)
            self.config_modified = self.path.stat().st_mtime_ns
            self.config_error = ''
        except Exception as exc:
            self.saved = {}
            self.config_modified = None
            self.config_error = 'Source config unavailable. '+str(exc)

    def request(self, payload):
        if self.busy:
            return 409, {'error': 'A tuning update is already pending'}
        if not self.values or time.monotonic()-self.updated > 5:
            return 503, {'error': 'Wait for current live tuning values'}
        if payload['mode'] == 'try':
            reason = self.can_apply()
            if reason:
                return 409, {'error': reason}
        elif not self.saved:
            return 503, {'error': self.config_error}
        self.pending = payload
        self.error = ''
        self.message = 'Reading current settings before '+('saving.' if payload['mode'] == 'save' else 'applying.')
        if self.operation is None:
            self.read()
        return 202, {'accepted': True}

    def read(self, phase='poll', **context):
        operation = dict(phase=phase, futures={}, received={}, started=time.monotonic(), **context)
        self.operation = operation
        self.last_read = time.monotonic()
        for name, (client, _) in self.clients.items():
            if not client.service_is_ready():
                self.read_failed(operation, 'Parameter server unavailable. '+name)
                return
        for name, (client, _) in self.clients.items():
            try:
                request = GetParameters.Request(names=PARAMETERS[name])
                future = client.call_async(request)
                operation['futures'][name] = future
                future.add_done_callback(lambda result, key=name: self.read_done(operation, key, result))
            except Exception as exc:
                self.read_failed(operation, 'Cannot request live tuning. '+str(exc))
                return

    def read_failed(self, operation, message):
        if self.operation is not operation:
            return
        self.operation = None
        for name, future in operation.get('futures', {}).items():
            if not future.done():
                self.clients[name][0].remove_pending_request(future)
                future.cancel()
        if operation['phase'] == 'verify':
            self.rollback(operation, message)
        else:
            self.pending = None
            self.error = message
            self.read_error = operation['phase'] == 'poll'
            self.message = 'Tuning readback failed.'
            if operation['phase'] == 'rollback_check':
                self.block_navigation = True

    def read_done(self, operation, name, future):
        if self.operation is not operation:
            return
        try:
            values = [parameter_value_to_python(value) for value in future.result().values]
            if len(values) != len(PARAMETERS[name]) or any(value is None for value in values):
                raise ValueError('Some parameters are not declared yet')
            operation['received'][name] = dict(zip(PARAMETERS[name], values))
            if len(operation['received']) != len(self.clients):
                return
            actual = operation['received']
            displayed = displayed_values(actual)
            # Validate array shape and numeric consistency before using the snapshot.
            parameter_values(displayed, actual)
            if self.read_error:
                self.error = ''
                self.read_error = False
            self.actual, self.values = actual, displayed
            self.updated = time.monotonic()
            self.node.recovery.speed = displayed['cruise_speed']
            self.operation = None
            phase = operation['phase']
            if phase == 'verify':
                if matches(actual, operation['desired']):
                    self.block_navigation = False
                    self.pending = None
                    self.message = 'Trial settings active for this bring-up.'
                    self.node.event(self.message)
                else:
                    self.rollback(operation, 'Nav2 readback did not match the requested settings')
            elif phase == 'rollback_check':
                self.pending = None
                self.block_navigation = not matches(actual, operation['previous'])
                self.error = operation['failure'] + (' Previous settings restored.' if not self.block_navigation else
                                                     ' Rollback is unconfirmed. Try settings again while stopped.')
                self.message = 'Tuning update failed.'
            elif self.pending:
                self.start_pending(actual)
            elif not self.error:
                self.message = 'Live values loaded. Trial changes last until bring-up restarts.'
        except Exception as exc:
            self.read_failed(operation, 'Cannot read live tuning. '+str(exc))

    def start_pending(self, actual):
        payload = self.pending
        if payload['mode'] == 'save':
            try:
                desired = parameter_values(payload['values'], actual)
                if not matches(actual, desired):
                    raise ValueError('Try these settings and wait for confirmation before making permanent')
                self.saved = save_config(self.path, payload['values'], payload['revision'])
                self.config_error = ''
                self.message = 'Permanent settings saved to '+str(self.path)
                self.node.event(self.message)
            except Exception as exc:
                self.error = str(exc)
                self.message = 'Permanent settings were not saved.'
                self.load_saved()
            self.pending = None
            return
        reason = self.can_apply()
        if reason:
            self.pending = None
            self.error = reason
            return
        previous = copy.deepcopy(actual)
        desired = parameter_values(payload['values'], actual)
        self.block_navigation = True
        self.set_next(dict(phase='apply', desired=desired, previous=previous,
                           remaining=list(self.clients)))

    def set_next(self, operation):
        if not operation['remaining']:
            if operation['phase'] == 'apply':
                self.read('verify', desired=operation['desired'], previous=operation['previous'])
            else:
                self.read('rollback_check', previous=operation['previous'], failure=operation['failure'])
            return
        name = operation['remaining'].pop(0)
        client = self.clients[name][1]
        operation.update(node=name, started=time.monotonic())
        self.operation = operation
        self.message = ('Restoring ' if operation['phase'] == 'rollback' else 'Applying ')+name+' settings.'
        if not client.service_is_ready():
            self.set_failed(operation, 'Parameter update service unavailable. '+name)
            return
        values = operation['previous'] if operation['phase'] == 'rollback' else operation['desired']
        try:
            request = SetParametersAtomically.Request(parameters=[
                Parameter(key, value=value).to_parameter_msg() for key, value in values[name].items()])
            operation['future'] = client.call_async(request)
            operation['future'].add_done_callback(lambda future: self.set_done(operation, future))
        except Exception as exc:
            self.set_failed(operation, str(exc))

    def set_done(self, operation, future):
        if self.operation is not operation:
            return
        try:
            result = future.result().result
            if not result.successful:
                raise ValueError(result.reason or 'Parameter update rejected')
            self.set_next(operation)
        except Exception as exc:
            self.set_failed(operation, str(exc))

    def set_failed(self, operation, message):
        if operation['phase'] == 'rollback':
            operation['failure'] += ' Rollback error for '+operation['node']+'. '+message
            self.set_next(operation)
        else:
            # A missing reply may follow a completed update. Restore that node too.
            self.rollback(operation, message)

    def rollback(self, operation, message):
        self.set_next(dict(phase='rollback', previous=operation['previous'], failure=message,
                           remaining=list(reversed(self.clients))))

    def tick(self):
        now = time.monotonic()
        operation = self.operation
        if operation and now-operation['started'] > 3:
            if operation['phase'] in ('apply', 'rollback'):
                future = operation.get('future')
                if future and not future.done():
                    self.clients[operation['node']][1].remove_pending_request(future)
                    self.operation = None
                    future.cancel()
                self.set_failed(operation, 'Parameter update acknowledgement timed out')
            else:
                self.read_failed(operation, 'Live tuning request timed out')
        elif operation is None and now-self.last_read >= 2:
            try:
                modified = self.path.stat().st_mtime_ns
            except OSError:
                modified = None
            if modified != self.config_modified:
                self.load_saved()
            self.read()

    def close(self):
        operation = self.operation
        self.operation = None
        self.pending = None
        if operation:
            futures = list(operation.get('futures', {}).values())
            if operation.get('future') is not None:
                futures.append(operation['future'])
            for future in futures:
                future.cancel()

    def snapshot(self):
        ready = bool(self.values) and time.monotonic()-self.updated <= 5
        reason = self.can_apply()
        return dict(schema=SETTINGS, values=self.values, saved=self.saved, config_path=str(self.path),
                    ready=ready, busy=self.busy, trial=bool(self.saved and not matches(self.values, self.saved['values'])), can_apply=ready and not reason and not self.busy,
                    blocked_reason=reason, error=self.error, config_error=self.config_error,
                    message=self.message, block_navigation=self.block_navigation)
