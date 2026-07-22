#!/usr/bin/env python3
import json
import os
import sys
from pathlib import Path

import requests


def extract_openai_text(data: dict) -> str:
    try:
        content = data["choices"][0]["message"]["content"]
    except (KeyError, IndexError, TypeError) as exc:
        raise ValueError(f"missing choices[0].message.content: {exc}") from exc

    if isinstance(content, str):
        return content.strip()

    if isinstance(content, list):
        parts = []
        for item in content:
            if isinstance(item, dict) and item.get("type") == "text":
                parts.append(str(item.get("text", "")))
        return "\n".join(parts).strip()

    return str(content).strip()


def main() -> int:
    if len(sys.argv) != 4:
        print(
            "usage: warehouse_ai_runner.py <prompt_path> <image_meta_path> <output_path>",
            file=sys.stderr,
        )
        return 1

    prompt_path = Path(sys.argv[1])
    image_meta_path = Path(sys.argv[2])
    output_path = Path(sys.argv[3])

    if not prompt_path.exists():
        print(f"prompt file not found: {prompt_path}", file=sys.stderr)
        return 1

    if not image_meta_path.exists():
        print(f"image meta file not found: {image_meta_path}", file=sys.stderr)
        return 1

    base_url = os.environ.get("RKLLM_BASE_URL", "http://127.0.0.1:8080/v1").strip().rstrip("/")
    api_key = os.environ.get("RKLLM_API_KEY", "not-required").strip() or "not-required"
    model_name = os.environ.get("RKLLM_MODEL", "model").strip() or "model"
    timeout_sec = float(os.environ.get("RKLLM_TIMEOUT_SEC", "120"))

    if not base_url:
        print("RKLLM_BASE_URL is empty", file=sys.stderr)
        return 1

    prompt_text = prompt_path.read_text(encoding="utf-8")

    try:
        image_meta = json.loads(image_meta_path.read_text(encoding="utf-8"))
    except Exception as e:
        print(f"failed to parse image meta json: {e}", file=sys.stderr)
        return 1

    mode = str(image_meta.get("mode", "")).strip()
    if mode and mode != "text_summary":
        print(
            "RKLLM text server only supports text_summary mode; visual mode needs a separate VLM service",
            file=sys.stderr,
        )
        return 1

    payload = {
        "model": model_name,
        "messages": [
            {
                "role": "user",
                "content": prompt_text,
            }
        ],
        "stream": False,
        "max_tokens": 160,
        "temperature": 0.2,
    }

    url = f"{base_url}/chat/completions"
    headers = {
        "Authorization": f"Bearer {api_key}",
        "Content-Type": "application/json",
    }

    try:
        response = requests.post(url, headers=headers, json=payload, timeout=timeout_sec)
    except Exception as e:
        print(f"request failed: {e}", file=sys.stderr)
        return 1

    if not response.ok:
        print(f"status={response.status_code}", file=sys.stderr)
        print(response.text, file=sys.stderr)
        return 1

    try:
        data = response.json()
    except Exception as e:
        print(f"invalid json response: {e}", file=sys.stderr)
        print(response.text, file=sys.stderr)
        return 1

    try:
        final_text = extract_openai_text(data)
    except Exception as e:
        print(f"failed to extract response text: {e}", file=sys.stderr)
        print(json.dumps(data, ensure_ascii=False), file=sys.stderr)
        return 1

    output_path.write_text(final_text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())