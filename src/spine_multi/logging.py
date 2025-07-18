import logging

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)

logger = logging.getLogger(__name__)


def get_logger(name=None, output="console", filename="app.log", level=logging.INFO):
    """
    Get a logger with flexible output options

    Args:
        name: Logger name (defaults to __name__)
        output: "console", "file", or "both"
        filename: Log file name (used when output is "file" or "both")
        level: Logging level (default: INFO)

    Returns:
        Logger instance
    """
    logger = logging.getLogger(name or __name__)

    # Clear existing handlers to avoid duplicates
    logger.handlers.clear()

    # Create formatter
    formatter = logging.Formatter(
        "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    )

    # Add console handler
    if output in ["console", "both"]:
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)

    # Add file handler
    if output in ["file", "both"]:
        file_handler = logging.FileHandler(filename)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

    logger.setLevel(level)
    return logger
