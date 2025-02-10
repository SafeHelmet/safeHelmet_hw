import ssd1306

DISPLAY = 1


class Display:
    def __init__(self, i2c_obj):
        if DISPLAY:
            self.last_status = []
            self.oled = ssd1306.SSD1306_I2C(128, 64, i2c_obj)  # Display OLED 128x64
        else:
            print("Adafruit SSD1306 is disabled by current configuration.")

    def write_text(self, text, x=0, y=0, clear=True):
        if DISPLAY:
            if [text, x, y, clear] != self.last_status:
                if clear:
                    self.oled.fill(0)  # Pulisce lo schermo

                lines = self._wrap_text(text) if len(text) > 14 else [text]
                for i, line in enumerate(lines):
                    self.oled.text(line, x, y + i * 10)  # Adjust y-offset for each line
                self.oled.show()
                self.last_status = [text, x, y, clear]

    def _wrap_text(self, text):
        """Wraps text to fit within 14 characters per line (more efficiently)."""
        lines = []
        start = 0
        while start < len(text):
            end = start + 14
            if end >= len(text):
                lines.append(text[start:].strip())
                break

            # Check for space to avoid cutting words
            if text[end] == ' ':  # ideal case
                lines.append(text[start:end].strip())
                start = end + 1
                continue

            # Check for space before the limit
            rfind_index = text.rfind(' ', start, end)
            if rfind_index != -1:
                lines.append(text[start:rfind_index].strip())
                start = rfind_index + 1
                continue

            # force split
            lines.append(text[start:end].strip())
            start = end
        return lines
