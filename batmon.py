import analogio
import board

vbat_sense = analogio.AnalogIn(board.VBAT_SENSE)

class BATMON:
    vbat = ((vbat_sense.value / 65535.0) * vbat_sense.reference_voltage)*(3.0/2.0)