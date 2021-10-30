import sys
import ctypes
from Phidget22.PhidgetSupport import PhidgetSupport
from Phidget22.Async import *
from Phidget22.LCDFont import LCDFont
from Phidget22.LCDPixelState import LCDPixelState
from Phidget22.LCDScreenSize import LCDScreenSize
from Phidget22.PhidgetException import PhidgetException

from Phidget22.Phidget import Phidget

class LCD(Phidget):

	def __init__(self):
		Phidget.__init__(self)
		self.handle = ctypes.c_void_p()
		self._setCharacterBitmap_async = None
		self._onsetCharacterBitmap_async = None
		self._clear_async = None
		self._onclear_async = None
		self._copy_async = None
		self._oncopy_async = None
		self._drawLine_async = None
		self._ondrawLine_async = None
		self._drawPixel_async = None
		self._ondrawPixel_async = None
		self._drawRect_async = None
		self._ondrawRect_async = None
		self._flush_async = None
		self._onflush_async = None
		self._setFrameBuffer_async = None
		self._onsetFrameBuffer_async = None
		self._saveFrameBuffer_async = None
		self._onsaveFrameBuffer_async = None
		self._writeBitmap_async = None
		self._onwriteBitmap_async = None
		self._writeText_async = None
		self._onwriteText_async = None

		__func = PhidgetSupport.getDll().PhidgetLCD_create
		__func.restype = ctypes.c_int32
		res = __func(ctypes.byref(self.handle))

		if res > 0:
			raise PhidgetException(res)

	def __del__(self):
		Phidget.__del__(self)

	def getBacklight(self):
		_Backlight = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetLCD_getBacklight
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_Backlight))

		if result > 0:
			raise PhidgetException(result)

		return _Backlight.value

	def setBacklight(self, Backlight):
		_Backlight = ctypes.c_double(Backlight)

		__func = PhidgetSupport.getDll().PhidgetLCD_setBacklight
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _Backlight)

		if result > 0:
			raise PhidgetException(result)


	def getMinBacklight(self):
		_MinBacklight = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetLCD_getMinBacklight
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinBacklight))

		if result > 0:
			raise PhidgetException(result)

		return _MinBacklight.value

	def getMaxBacklight(self):
		_MaxBacklight = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetLCD_getMaxBacklight
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxBacklight))

		if result > 0:
			raise PhidgetException(result)

		return _MaxBacklight.value

	def setCharacterBitmap(self, font, character, bitmap):
		_font = ctypes.c_int(font)
		_character = ctypes.create_string_buffer(character.encode('utf-8'))
		_bitmap = (ctypes.c_uint8 * len(bitmap))(*bitmap)

		__func = PhidgetSupport.getDll().PhidgetLCD_setCharacterBitmap
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _font, ctypes.byref(_character), ctypes.byref(_bitmap))

		if result > 0:
			raise PhidgetException(result)


	def setCharacterBitmap_async(self, font, character, bitmap, asyncHandler):
		_font = ctypes.c_int(font)
		_character = ctypes.create_string_buffer(character.encode('utf-8'))
		_bitmap = (ctypes.c_uint8 * len(bitmap))(*bitmap)

		_ctx = ctypes.c_void_p()
		if asyncHandler != None:
			_ctx = ctypes.c_void_p(AsyncSupport.add(asyncHandler, self))
		_asyncHandler = AsyncSupport.getCallback()

		__func = PhidgetSupport.getDll().PhidgetLCD_setCharacterBitmap_async
		__func(self.handle, _font, ctypes.byref(_character), ctypes.byref(_bitmap), _asyncHandler, _ctx)


	def getMaxCharacters(self, font):
		_font = ctypes.c_int(font)
		_maxCharacters = ctypes.c_int()

		__func = PhidgetSupport.getDll().PhidgetLCD_getMaxCharacters
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _font, ctypes.byref(_maxCharacters))

		if result > 0:
			raise PhidgetException(result)

		return _maxCharacters.value

	def clear(self):
		__func = PhidgetSupport.getDll().PhidgetLCD_clear
		__func.restype = ctypes.c_int32
		result = __func(self.handle)

		if result > 0:
			raise PhidgetException(result)


	def clear_async(self, asyncHandler):

		_ctx = ctypes.c_void_p()
		if asyncHandler != None:
			_ctx = ctypes.c_void_p(AsyncSupport.add(asyncHandler, self))
		_asyncHandler = AsyncSupport.getCallback()

		__func = PhidgetSupport.getDll().PhidgetLCD_clear_async
		__func(self.handle, _asyncHandler, _ctx)


	def getContrast(self):
		_Contrast = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetLCD_getContrast
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_Contrast))

		if result > 0:
			raise PhidgetException(result)

		return _Contrast.value

	def setContrast(self, Contrast):
		_Contrast = ctypes.c_double(Contrast)

		__func = PhidgetSupport.getDll().PhidgetLCD_setContrast
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _Contrast)

		if result > 0:
			raise PhidgetException(result)


	def getMinContrast(self):
		_MinContrast = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetLCD_getMinContrast
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinContrast))

		if result > 0:
			raise PhidgetException(result)

		return _MinContrast.value

	def getMaxContrast(self):
		_MaxContrast = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetLCD_getMaxContrast
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxContrast))

		if result > 0:
			raise PhidgetException(result)

		return _MaxContrast.value

	def copy(self, sourceFramebuffer, destFramebuffer, sourceX1, sourceY1, sourceX2, sourceY2, destX, destY, inverted):
		_sourceFramebuffer = ctypes.c_int(sourceFramebuffer)
		_destFramebuffer = ctypes.c_int(destFramebuffer)
		_sourceX1 = ctypes.c_int(sourceX1)
		_sourceY1 = ctypes.c_int(sourceY1)
		_sourceX2 = ctypes.c_int(sourceX2)
		_sourceY2 = ctypes.c_int(sourceY2)
		_destX = ctypes.c_int(destX)
		_destY = ctypes.c_int(destY)
		_inverted = ctypes.c_int(inverted)

		__func = PhidgetSupport.getDll().PhidgetLCD_copy
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _sourceFramebuffer, _destFramebuffer, _sourceX1, _sourceY1, _sourceX2, _sourceY2, _destX, _destY, _inverted)

		if result > 0:
			raise PhidgetException(result)


	def copy_async(self, sourceFramebuffer, destFramebuffer, sourceX1, sourceY1, sourceX2, sourceY2, destX, destY, inverted, asyncHandler):
		_sourceFramebuffer = ctypes.c_int(sourceFramebuffer)
		_destFramebuffer = ctypes.c_int(destFramebuffer)
		_sourceX1 = ctypes.c_int(sourceX1)
		_sourceY1 = ctypes.c_int(sourceY1)
		_sourceX2 = ctypes.c_int(sourceX2)
		_sourceY2 = ctypes.c_int(sourceY2)
		_destX = ctypes.c_int(destX)
		_destY = ctypes.c_int(destY)
		_inverted = ctypes.c_int(inverted)

		_ctx = ctypes.c_void_p()
		if asyncHandler != None:
			_ctx = ctypes.c_void_p(AsyncSupport.add(asyncHandler, self))
		_asyncHandler = AsyncSupport.getCallback()

		__func = PhidgetSupport.getDll().PhidgetLCD_copy_async
		__func(self.handle, _sourceFramebuffer, _destFramebuffer, _sourceX1, _sourceY1, _sourceX2, _sourceY2, _destX, _destY, _inverted, _asyncHandler, _ctx)


	def getCursorBlink(self):
		_CursorBlink = ctypes.c_int()

		__func = PhidgetSupport.getDll().PhidgetLCD_getCursorBlink
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_CursorBlink))

		if result > 0:
			raise PhidgetException(result)

		return _CursorBlink.value

	def setCursorBlink(self, CursorBlink):
		_CursorBlink = ctypes.c_int(CursorBlink)

		__func = PhidgetSupport.getDll().PhidgetLCD_setCursorBlink
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _CursorBlink)

		if result > 0:
			raise PhidgetException(result)


	def getCursorOn(self):
		_CursorOn = ctypes.c_int()

		__func = PhidgetSupport.getDll().PhidgetLCD_getCursorOn
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_CursorOn))

		if result > 0:
			raise PhidgetException(result)

		return _CursorOn.value

	def setCursorOn(self, CursorOn):
		_CursorOn = ctypes.c_int(CursorOn)

		__func = PhidgetSupport.getDll().PhidgetLCD_setCursorOn
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _CursorOn)

		if result > 0:
			raise PhidgetException(result)


	def drawLine(self, x1, y1, x2, y2):
		_x1 = ctypes.c_int(x1)
		_y1 = ctypes.c_int(y1)
		_x2 = ctypes.c_int(x2)
		_y2 = ctypes.c_int(y2)

		__func = PhidgetSupport.getDll().PhidgetLCD_drawLine
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _x1, _y1, _x2, _y2)

		if result > 0:
			raise PhidgetException(result)


	def drawLine_async(self, x1, y1, x2, y2, asyncHandler):
		_x1 = ctypes.c_int(x1)
		_y1 = ctypes.c_int(y1)
		_x2 = ctypes.c_int(x2)
		_y2 = ctypes.c_int(y2)

		_ctx = ctypes.c_void_p()
		if asyncHandler != None:
			_ctx = ctypes.c_void_p(AsyncSupport.add(asyncHandler, self))
		_asyncHandler = AsyncSupport.getCallback()

		__func = PhidgetSupport.getDll().PhidgetLCD_drawLine_async
		__func(self.handle, _x1, _y1, _x2, _y2, _asyncHandler, _ctx)


	def drawPixel(self, x, y, pixelState):
		_x = ctypes.c_int(x)
		_y = ctypes.c_int(y)
		_pixelState = ctypes.c_int(pixelState)

		__func = PhidgetSupport.getDll().PhidgetLCD_drawPixel
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _x, _y, _pixelState)

		if result > 0:
			raise PhidgetException(result)


	def drawPixel_async(self, x, y, pixelState, asyncHandler):
		_x = ctypes.c_int(x)
		_y = ctypes.c_int(y)
		_pixelState = ctypes.c_int(pixelState)

		_ctx = ctypes.c_void_p()
		if asyncHandler != None:
			_ctx = ctypes.c_void_p(AsyncSupport.add(asyncHandler, self))
		_asyncHandler = AsyncSupport.getCallback()

		__func = PhidgetSupport.getDll().PhidgetLCD_drawPixel_async
		__func(self.handle, _x, _y, _pixelState, _asyncHandler, _ctx)


	def drawRect(self, x1, y1, x2, y2, filled, inverted):
		_x1 = ctypes.c_int(x1)
		_y1 = ctypes.c_int(y1)
		_x2 = ctypes.c_int(x2)
		_y2 = ctypes.c_int(y2)
		_filled = ctypes.c_int(filled)
		_inverted = ctypes.c_int(inverted)

		__func = PhidgetSupport.getDll().PhidgetLCD_drawRect
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _x1, _y1, _x2, _y2, _filled, _inverted)

		if result > 0:
			raise PhidgetException(result)


	def drawRect_async(self, x1, y1, x2, y2, filled, inverted, asyncHandler):
		_x1 = ctypes.c_int(x1)
		_y1 = ctypes.c_int(y1)
		_x2 = ctypes.c_int(x2)
		_y2 = ctypes.c_int(y2)
		_filled = ctypes.c_int(filled)
		_inverted = ctypes.c_int(inverted)

		_ctx = ctypes.c_void_p()
		if asyncHandler != None:
			_ctx = ctypes.c_void_p(AsyncSupport.add(asyncHandler, self))
		_asyncHandler = AsyncSupport.getCallback()

		__func = PhidgetSupport.getDll().PhidgetLCD_drawRect_async
		__func(self.handle, _x1, _y1, _x2, _y2, _filled, _inverted, _asyncHandler, _ctx)


	def flush(self):
		__func = PhidgetSupport.getDll().PhidgetLCD_flush
		__func.restype = ctypes.c_int32
		result = __func(self.handle)

		if result > 0:
			raise PhidgetException(result)


	def flush_async(self, asyncHandler):

		_ctx = ctypes.c_void_p()
		if asyncHandler != None:
			_ctx = ctypes.c_void_p(AsyncSupport.add(asyncHandler, self))
		_asyncHandler = AsyncSupport.getCallback()

		__func = PhidgetSupport.getDll().PhidgetLCD_flush_async
		__func(self.handle, _asyncHandler, _ctx)


	def getFontSize(self, font):
		_font = ctypes.c_int(font)
		_width = ctypes.c_int()
		_height = ctypes.c_int()

		__func = PhidgetSupport.getDll().PhidgetLCD_getFontSize
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _font, ctypes.byref(_width), ctypes.byref(_height))

		if result > 0:
			raise PhidgetException(result)

		return _width.value, _height.value

	def setFontSize(self, font, width, height):
		_font = ctypes.c_int(font)
		_width = ctypes.c_int(width)
		_height = ctypes.c_int(height)

		__func = PhidgetSupport.getDll().PhidgetLCD_setFontSize
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _font, _width, _height)

		if result > 0:
			raise PhidgetException(result)


	def getFrameBuffer(self):
		_FrameBuffer = ctypes.c_int()

		__func = PhidgetSupport.getDll().PhidgetLCD_getFrameBuffer
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_FrameBuffer))

		if result > 0:
			raise PhidgetException(result)

		return _FrameBuffer.value

	def setFrameBuffer(self, FrameBuffer):
		_FrameBuffer = ctypes.c_int(FrameBuffer)

		__func = PhidgetSupport.getDll().PhidgetLCD_setFrameBuffer
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _FrameBuffer)

		if result > 0:
			raise PhidgetException(result)


	def setFrameBuffer_async(self, FrameBuffer, asyncHandler):
		_FrameBuffer = ctypes.c_int(FrameBuffer)

		_ctx = ctypes.c_void_p()
		if asyncHandler != None:
			_ctx = ctypes.c_void_p(AsyncSupport.add(asyncHandler, self))
		_asyncHandler = AsyncSupport.getCallback()

		__func = PhidgetSupport.getDll().PhidgetLCD_setFrameBuffer_async
		__func(self.handle, _FrameBuffer, _asyncHandler, _ctx)


	def getHeight(self):
		_Height = ctypes.c_int()

		__func = PhidgetSupport.getDll().PhidgetLCD_getHeight
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_Height))

		if result > 0:
			raise PhidgetException(result)

		return _Height.value

	def initialize(self):
		__func = PhidgetSupport.getDll().PhidgetLCD_initialize
		__func.restype = ctypes.c_int32
		result = __func(self.handle)

		if result > 0:
			raise PhidgetException(result)


	def saveFrameBuffer(self, frameBuffer):
		_frameBuffer = ctypes.c_int(frameBuffer)

		__func = PhidgetSupport.getDll().PhidgetLCD_saveFrameBuffer
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _frameBuffer)

		if result > 0:
			raise PhidgetException(result)


	def saveFrameBuffer_async(self, frameBuffer, asyncHandler):
		_frameBuffer = ctypes.c_int(frameBuffer)

		_ctx = ctypes.c_void_p()
		if asyncHandler != None:
			_ctx = ctypes.c_void_p(AsyncSupport.add(asyncHandler, self))
		_asyncHandler = AsyncSupport.getCallback()

		__func = PhidgetSupport.getDll().PhidgetLCD_saveFrameBuffer_async
		__func(self.handle, _frameBuffer, _asyncHandler, _ctx)


	def getScreenSize(self):
		_ScreenSize = ctypes.c_int()

		__func = PhidgetSupport.getDll().PhidgetLCD_getScreenSize
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_ScreenSize))

		if result > 0:
			raise PhidgetException(result)

		return _ScreenSize.value

	def setScreenSize(self, ScreenSize):
		_ScreenSize = ctypes.c_int(ScreenSize)

		__func = PhidgetSupport.getDll().PhidgetLCD_setScreenSize
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _ScreenSize)

		if result > 0:
			raise PhidgetException(result)


	def getSleeping(self):
		_Sleeping = ctypes.c_int()

		__func = PhidgetSupport.getDll().PhidgetLCD_getSleeping
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_Sleeping))

		if result > 0:
			raise PhidgetException(result)

		return _Sleeping.value

	def setSleeping(self, Sleeping):
		_Sleeping = ctypes.c_int(Sleeping)

		__func = PhidgetSupport.getDll().PhidgetLCD_setSleeping
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _Sleeping)

		if result > 0:
			raise PhidgetException(result)


	def getWidth(self):
		_Width = ctypes.c_int()

		__func = PhidgetSupport.getDll().PhidgetLCD_getWidth
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_Width))

		if result > 0:
			raise PhidgetException(result)

		return _Width.value

	def writeBitmap(self, xPosition, yPosition, xSize, ySize, bitmap):
		_xPosition = ctypes.c_int(xPosition)
		_yPosition = ctypes.c_int(yPosition)
		_xSize = ctypes.c_int(xSize)
		_ySize = ctypes.c_int(ySize)
		_bitmap = (ctypes.c_uint8 * len(bitmap))(*bitmap)

		__func = PhidgetSupport.getDll().PhidgetLCD_writeBitmap
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _xPosition, _yPosition, _xSize, _ySize, ctypes.byref(_bitmap))

		if result > 0:
			raise PhidgetException(result)


	def writeBitmap_async(self, xPosition, yPosition, xSize, ySize, bitmap, asyncHandler):
		_xPosition = ctypes.c_int(xPosition)
		_yPosition = ctypes.c_int(yPosition)
		_xSize = ctypes.c_int(xSize)
		_ySize = ctypes.c_int(ySize)
		_bitmap = (ctypes.c_uint8 * len(bitmap))(*bitmap)

		_ctx = ctypes.c_void_p()
		if asyncHandler != None:
			_ctx = ctypes.c_void_p(AsyncSupport.add(asyncHandler, self))
		_asyncHandler = AsyncSupport.getCallback()

		__func = PhidgetSupport.getDll().PhidgetLCD_writeBitmap_async
		__func(self.handle, _xPosition, _yPosition, _xSize, _ySize, ctypes.byref(_bitmap), _asyncHandler, _ctx)


	def writeText(self, font, xPosition, yPosition, text):
		_font = ctypes.c_int(font)
		_xPosition = ctypes.c_int(xPosition)
		_yPosition = ctypes.c_int(yPosition)
		_text = ctypes.create_string_buffer(text.encode('utf-8'))

		__func = PhidgetSupport.getDll().PhidgetLCD_writeText
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _font, _xPosition, _yPosition, ctypes.byref(_text))

		if result > 0:
			raise PhidgetException(result)


	def writeText_async(self, font, xPosition, yPosition, text, asyncHandler):
		_font = ctypes.c_int(font)
		_xPosition = ctypes.c_int(xPosition)
		_yPosition = ctypes.c_int(yPosition)
		_text = ctypes.create_string_buffer(text.encode('utf-8'))

		_ctx = ctypes.c_void_p()
		if asyncHandler != None:
			_ctx = ctypes.c_void_p(AsyncSupport.add(asyncHandler, self))
		_asyncHandler = AsyncSupport.getCallback()

		__func = PhidgetSupport.getDll().PhidgetLCD_writeText_async
		__func(self.handle, _font, _xPosition, _yPosition, ctypes.byref(_text), _asyncHandler, _ctx)

