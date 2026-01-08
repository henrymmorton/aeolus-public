import os
import logging
import logging.config
from datetime import datetime

from .constants import LOG_DIR

timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
log_file = os.path.join(LOG_DIR, f"aeolus_{timestamp}.log")

LOGGING_CONFIG = {
    "version": 1,
    "disable_existing_loggers": False,
    "formatters": {
        "default": {
            "format": "[%(asctime)s] [%(levelname)s] %(name)s: %(message)s"
        },
    },
    "handlers": {
        "console": {
            "class": "logging.StreamHandler",
            "formatter": "default",
            "level": "WARNING",
        },
        "file": {
            "class": "logging.FileHandler",
            "filename": log_file,
            "formatter": "default",
            "level": "INFO",
        }
    },
    "loggers": {
        "core":{
            "level": "WARNING",
            "handlers": ["console", "file"],
            "propagate": False,
        },
    },
    "root": {
        "handlers": ["console", "file"],
        "level": "WARNING",
    },
}


PROCESS_LOGGING_CONFIG = {
    "version": 1,
    "disable_existing_loggers": False,
    "loggers": {
        "core":{
            "level": "WARNING",
            "handlers": [],
        },
    },
}

PROCESS_LOGGING_CONFIG_OWN_FILE = {
    "version": 1,
    "disable_existing_loggers": False,
    "formatters": {
        "default": {
            "format": "[%(asctime)s] [%(levelname)s] %(name)s: %(message)s"
        },
    },
    "handlers": {
        "file": {
            "class": "logging.FileHandler",
            "filename": log_file,
            "formatter": "default",
            "level": "WARNING",
        }
    },
    "loggers": {
        "core":{
            "level": "WARNING",
            "handlers": ["file"],
        },
    },
    "root": {
        "handlers": ["file"],
        "level": "INFO",
    },
}
