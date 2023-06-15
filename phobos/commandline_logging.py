import logging
import sys

SUPPORTED_LEVELS = ["DEBUG", "INFO", "WARNING", "ERROR"]
LOGGER_NAME = "phobos_log"
BASE_LOG_LEVEL = "WARNING"
LOG_FILE_CONVENTION = None

#The background is set with 40 plus the number of the color, and the foreground with 30
BLACK, RED, GREEN, YELLOW, BLUE, MAGENTA, CYAN, WHITE = range(8)

#These are the sequences need to get colored ouput
RESET_SEQ = "\033[0m"
COLOR_SEQ = "\033[1;%dm"
BOLD_SEQ = "\033[1m"
COLORS = {
    'WARNING': YELLOW,
    'INFO': WHITE,
    'DEBUG': BLUE,
    'CRITICAL': RED,
    'ERROR': RED
}


class ColoredFormatter(logging.Formatter):
    def __init__(self, msg, use_color = True):
        logging.Formatter.__init__(self, msg)
        self.use_color = use_color

    def format(self, record):
        levelname = record.levelname
        if self.use_color and levelname in COLORS:
            return BOLD_SEQ + COLOR_SEQ % (30 + COLORS[levelname]) + levelname + ": " + RESET_SEQ + logging.Formatter.format(self, record)
        return logging.Formatter.format(self, record)


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
        sh_formatter = ColoredFormatter("%(message)s")
        formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
        sh = logging.StreamHandler(sys.stdout)
        sh.setFormatter(sh_formatter)
        logger.handlers.clear()
        logger.addHandler(sh)
        if file_name:
            fh = logging.FileHandler(file_name)
            fh.setFormatter(formatter)
            logger.addHandler(fh)
        return logger


def get_logger(module_name):
    child_logger = logging.getLogger(LOGGER_NAME).getChild(module_name)
    child_logger.setLevel(logging.NOTSET)  # This ensures that child logger inherits from parent logger
    return child_logger
