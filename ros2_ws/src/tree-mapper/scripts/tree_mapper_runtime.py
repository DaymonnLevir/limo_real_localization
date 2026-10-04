#!/usr/bin/env python3

import json
import os
import re
import time

from tree_mapper_ros2 import package_share_dir


AUTO_TOKEN = "auto"


def normalize_optional_path(path_value):
    if path_value is None:
        return ""
    text = str(path_value).strip()
    if (not text) or text.lower() == AUTO_TOKEN:
        return ""
    return os.path.abspath(os.path.expanduser(text))


def ensure_dir(path_value):
    if path_value and (not os.path.exists(path_value)):
        os.makedirs(path_value)


def ensure_parent_dir(path_value):
    if not path_value:
        return
    ensure_dir(os.path.dirname(path_value))


def sanitize_token(text, fallback):
    candidate = str(text).strip()
    if not candidate:
        candidate = fallback
    candidate = re.sub(r"[^A-Za-z0-9_.-]+", "_", candidate)
    candidate = candidate.strip("._-")
    return candidate or fallback


def timestamp_token():
    return time.strftime("%Y%m%d_%H%M%S", time.localtime())


def precise_timestamp_token():
    now = time.time()
    local = time.localtime(now)
    millis = int((now - int(now)) * 1000.0)
    return time.strftime("%Y%m%d_%H%M%S", local) + "_%03d" % millis


def runtime_state_prefix(state_namespace):
    raw = str(state_namespace).strip()
    if not raw:
        raw = "/tree_mapper_runtime"
    if not raw.startswith("/"):
        raw = "/" + raw
    return raw.rstrip("/")


def _default_runs_root(package_name):
    explicit_root = normalize_optional_path(os.environ.get("TREE_MAPPER_RUNS_DIR", ""))
    if explicit_root:
        return explicit_root
    return os.path.join(os.path.expanduser("~"), "%s_runs" % package_name)


def start_new_run(package_name, requested_run_output_dir=""):
    run_id = precise_timestamp_token()
    explicit_run_dir = normalize_optional_path(requested_run_output_dir)
    run_dir = explicit_run_dir if explicit_run_dir else os.path.join(_default_runs_root(package_name), run_id)

    ensure_dir(run_dir)
    snapshots_dir = os.path.join(run_dir, "snapshots")
    ensure_dir(snapshots_dir)
    package_root = package_share_dir(package_name)

    return {
        "package_root": package_root,
        "run_id": run_id,
        "run_dir": run_dir,
        "snapshots_dir": snapshots_dir,
    }


def encode_reset_payload(runtime):
    payload = {
        "run_id": str(runtime.get("run_id", "")).strip(),
        "run_dir": normalize_optional_path(runtime.get("run_dir", "")),
        "snapshots_dir": normalize_optional_path(runtime.get("snapshots_dir", "")),
    }
    return json.dumps(payload, sort_keys=True)


def decode_reset_payload(payload_text):
    text = str(payload_text).strip()
    if not text:
        return {}
    try:
        payload = json.loads(text)
    except Exception:
        return {"run_id": text}
    if not isinstance(payload, dict):
        return {}
    return {
        "run_id": str(payload.get("run_id", "")).strip(),
        "run_dir": normalize_optional_path(payload.get("run_dir", "")),
        "snapshots_dir": normalize_optional_path(payload.get("snapshots_dir", "")),
    }


def resolve_runtime_context(node, package_name, requested_run_output_dir, requested_run_id, state_namespace):
    prefix = runtime_state_prefix(state_namespace)
    explicit_run_dir = normalize_optional_path(requested_run_output_dir)
    explicit_run_id = sanitize_token(requested_run_id, timestamp_token()) if str(requested_run_id).strip() else ""
    env_run_id = sanitize_token(os.environ.get("TREE_MAPPER_RUN_ID", "").strip(), timestamp_token())
    env_run_dir = normalize_optional_path(os.environ.get("TREE_MAPPER_RUN_DIR", ""))

    run_id = explicit_run_id if explicit_run_id else env_run_id
    if not run_id:
        run_id = timestamp_token()

    if explicit_run_dir:
        run_dir = explicit_run_dir
    elif env_run_dir:
        run_dir = env_run_dir
    else:
        run_dir = os.path.join(_default_runs_root(package_name), run_id)

    ensure_dir(run_dir)
    snapshots_dir = os.path.join(run_dir, "snapshots")
    ensure_dir(snapshots_dir)
    package_root = package_share_dir(package_name)

    return {
        "package_root": package_root,
        "run_id": run_id,
        "run_dir": run_dir,
        "snapshots_dir": snapshots_dir,
        "state_namespace": prefix,
    }


def resolve_output_file(explicit_path, run_dir, default_filename):
    normalized = normalize_optional_path(explicit_path)
    if normalized:
        ensure_parent_dir(normalized)
        return normalized
    resolved = os.path.join(run_dir, default_filename)
    ensure_parent_dir(resolved)
    return resolved
