from sync_resilience import install_for_hardware_runtime

install_for_hardware_runtime()

from navigation_app import run_app


if __name__ == "__main__":
    run_app("hardware")
