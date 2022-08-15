import logging
import sys
from ..defs import BASE_LOG_LEVEL, LOG_FILE_CONVENTION

SUPPORTED_LEVELS = ["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"]
LOGGER_NAME = "phobos_log"


def setup_logger_level(log_level=BASE_LOG_LEVEL, file_name=LOG_FILE_CONVENTION):
    """
    Setup tool for inbuilt logging module, set the logging level for output and the filename and you are ready to go.
    """
    logger = logging.getLogger(LOGGER_NAME)
    if not log_level.upper() in SUPPORTED_LEVELS:
        print(f"Not supported logging level. Supported are {SUPPORTED_LEVELS}")
        return None
    else:
        if log_level.upper() == "DEBUG":
            logger.setLevel(logging.DEBUG)
        elif log_level.upper() == "INFO":
            logger.setLevel(logging.INFO)
        elif log_level.upper() == "WARNING":
            logger.setLevel(logging.WARNING)
        elif log_level.upper() == "ERROR":
            logger.setLevel(logging.ERROR)
        elif log_level.upper() == "CRITICAL":
            logger.setLevel(logging.CRITICAL)
        formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
        sh = logging.StreamHandler(sys.stdout)
        sh.setFormatter(formatter)
        logger.handlers.clear()
        logger.addHandler(sh)
        if file_name:
            fh = logging.FileHandler(file_name)
            fh.setFormatter(formatter)
            logger.addHandler(fh)
        return logger


def get_logger(module_name):
    return logging.getLogger(LOGGER_NAME).getChild(module_name)
