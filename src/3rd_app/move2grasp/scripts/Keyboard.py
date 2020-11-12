from pynput import keyboard

class Keyboard:
    def __init__(self):
        self.pressedKey = {}
        self.Key = keyboard.Key

        def on_press(key):
            if type(key) is keyboard.KeyCode:
                self.pressedKey[key.char.lower()] = True
            else:
                self.pressedKey[key] = True

        def on_release(key):
            if type(key) is keyboard.KeyCode:
                self.pressedKey[key.char.lower()] = False
            else:
                self.pressedKey[key] = False

        listener = keyboard.Listener(
            on_press=on_press,
            on_release=on_release)
        #listener.join()
        listener.start()

    def isPress(self, key):
        return self.pressedKey.get(key, False)

    def isPressAndWaitForRelease(self, key):
        if not self.isPress(key):
            return False
        self.waitForRelease(key)
        return True

    def waitForPress(self, key):
        while not self.isPress(key):
            pass

    def waitForRelease(self, key):
        while self.isPress(key):
            pass

    def wait(self, key):
        self.waitForPress(key)
        self.waitForRelease(key)