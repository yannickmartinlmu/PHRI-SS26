#!/usr/bin/env python3
"""Small Home Assistant REST client used by BrewBot actuators."""

import json
from urllib.error import HTTPError, URLError
from urllib.parse import urljoin
from urllib.request import Request, urlopen

HOME_ASSISTANT_URL = "http://10.163.18.107:8123"
HOME_ASSISTANT_TOKEN = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJmZmY2MzJkZDdiNzQ0M2VlOGM1NTA0M2ZiNmM0ZDcxOSIsImlhdCI6MTc4NDI5MTY3NiwiZXhwIjoyMDk5NjUxNjc2fQ.wXIpLtHo3JDWykV0ghJeHe7hZvgy4WRFHMej06WMRzM"


class HomeAssistantError(RuntimeError):
    """Raised when Home Assistant is unavailable or returns an error."""


class HomeAssistantClient:
    def __init__(self, base_url=None, token=None, timeout=10.0):
        self.base_url = (base_url or HOME_ASSISTANT_URL).rstrip("/")
        self.token = token or HOME_ASSISTANT_TOKEN
        self.timeout = float(timeout)
        if not self.base_url:
            raise HomeAssistantError("HOME_ASSISTANT_URL is not set")
        if not self.token:
            raise HomeAssistantError("HOME_ASSISTANT_TOKEN is not set")

    def get_state(self, entity_id):
        return self._request("GET", f"/api/states/{entity_id}")

    def call_service(self, domain, service, body):
        return self._request("POST", f"/api/services/{domain}/{service}", body)

    def _request(self, method, path, body=None):
        url = urljoin(f"{self.base_url}/", path.lstrip("/"))
        data = None
        if body is not None:
            data = json.dumps(body).encode("utf-8")
        req = Request(
            url,
            data=data,
            method=method,
            headers={
                "Authorization": f"Bearer {self.token}",
                "Content-Type": "application/json",
            },
        )
        try:
            with urlopen(req, timeout=self.timeout) as response:
                payload = response.read().decode("utf-8")
        except HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise HomeAssistantError(f"Home Assistant HTTP {exc.code}: {detail}") from exc
        except URLError as exc:
            raise HomeAssistantError(f"Home Assistant request failed: {exc.reason}") from exc
        except TimeoutError as exc:
            raise HomeAssistantError("Home Assistant request timed out") from exc
        if not payload:
            return None
        return json.loads(payload)
