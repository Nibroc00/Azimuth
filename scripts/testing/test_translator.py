import sys
sys.path.insert(0, '../')
import Azimuth.src.Translator as Translator

trans = Translator.translator(40.819375, -96.706161)
trans.updatepos(0, 0, 0)

print(type(trans))
print(trans.getlocalloc())
print(trans.azimuth(5, 5))
