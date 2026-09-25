#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Small Tk tool that sends a custom CAN frame once or periodically over SocketCAN."""

import tkinter as tk
from tkinter import Button, Checkbutton, Entry, IntVar, Label, Text

import can

STANDARD_ID_MAX = 0x7FF
EXTENDED_ID_MAX = 0x1FFFFFFF
LOOP_PERIOD_MS = 1000


def parse_frame(can_id_text, payload_texts):
    """Return (arbitration_id, is_extended_id, data) or raise ValueError."""
    if not can_id_text:
        raise ValueError('CAN ID is empty')
    can_id = int(can_id_text, 16)
    if can_id > EXTENDED_ID_MAX:
        raise ValueError(f'CAN ID 0x{can_id:X} exceeds 29 bits')
    # Leave empty trailing bytes out, so shorter frames (DLC < 8) can be sent.
    data = [int(text, 16) for text in payload_texts if text]
    if any(byte > 0xFF for byte in data):
        raise ValueError('payload bytes must be 00..FF')
    return can_id, can_id > STANDARD_ID_MAX, data


class CANMessageSender:
    """Tk front end; keeps one SocketCAN bus open per interface name."""

    def __init__(self, master):
        self.master = master
        master.title('Custom CAN Message Sender')

        self.label_interface = Label(master, text='Interface:')
        self.label_id = Label(master, text='CAN ID:')
        self.label_payload = Label(master, text='Payload:')
        self.entry_interface = Entry(master)
        validate = (master.register(self.validate_hex), '%P')
        self.entry_id = Entry(master, validate='key', validatecommand=validate)
        self.entry_payload = [Entry(master, width=3, validate='key', validatecommand=validate)
                              for _ in range(8)]

        self.label_interface.grid(row=0, column=0, padx=10, pady=10)
        self.label_id.grid(row=0, column=2, padx=2, pady=2)
        self.label_payload.grid(row=0, column=4, padx=10, pady=10)
        self.entry_interface.grid(row=0, column=1, padx=10, pady=10)
        self.entry_id.grid(row=0, column=3, padx=2, pady=2)
        for i, entry in enumerate(self.entry_payload):
            entry.grid(row=0, column=5 + i, padx=2, pady=10)

        self.text_output = Text(master, height=10, width=60)
        self.text_output.grid(row=1, columnspan=12, padx=10, pady=10)

        self.send_button = Button(master, text='Send Message', command=self.send_message)
        self.send_button.grid(row=2, columnspan=12, pady=10)

        self.loop_var = IntVar(value=0)
        self.loop_button = Checkbutton(master, text='Loop', variable=self.loop_var,
                                       command=self.toggle_loop)
        self.loop_button.grid(row=2, column=1, pady=10)

        self.loop_task_id = None
        self.bus = None
        self.bus_channel = None
        master.protocol('WM_DELETE_WINDOW', self.close)

    def log(self, text):
        self.text_output.insert(tk.END, text + '\n')
        self.text_output.yview(tk.END)

    def get_bus(self, channel):
        if self.bus is None or self.bus_channel != channel:
            self.close_bus()
            self.bus = can.Bus(channel=channel, interface='socketcan')
            self.bus_channel = channel
        return self.bus

    def close_bus(self):
        if self.bus is not None:
            try:
                self.bus.shutdown()
            finally:
                self.bus = None
                self.bus_channel = None

    def send_message(self):
        """Send one frame; any error is reported and never escapes the Tk callback."""
        try:
            channel = self.entry_interface.get().strip()
            if not channel:
                raise ValueError('interface is empty')
            can_id, extended, data = parse_frame(
                self.entry_id.get(), [entry.get() for entry in self.entry_payload])
            message = can.Message(arbitration_id=can_id, is_extended_id=extended, data=data)
            self.get_bus(channel).send(message)
            width = 8 if extended else 3
            self.log(f"{channel} {can_id:0{width}X} {' '.join(f'{b:02X}' for b in data)}")
        except (ValueError, OSError, can.CanError) as error:
            self.log(f'Error: {error}')
            # A failed bus (for example the interface went down) is reopened next time.
            if not isinstance(error, ValueError):
                self.close_bus()

    def toggle_loop(self):
        self.cancel_loop()
        if self.loop_var.get():
            self.loop_task_id = self.master.after(LOOP_PERIOD_MS, self.send_loop_message)

    def cancel_loop(self):
        if self.loop_task_id is not None:
            self.master.after_cancel(self.loop_task_id)
            self.loop_task_id = None

    def send_loop_message(self):
        self.loop_task_id = None
        try:
            self.send_message()
        except Exception as error:  # noqa: B902 - the periodic loop must survive anything.
            self.log(f'Error: {error}')
        finally:
            if self.loop_var.get():
                self.loop_task_id = self.master.after(LOOP_PERIOD_MS, self.send_loop_message)

    def close(self):
        self.cancel_loop()
        self.close_bus()
        self.master.destroy()

    @staticmethod
    def validate_hex(value):
        return all(c in '0123456789ABCDEFabcdef' for c in value)


if __name__ == '__main__':
    root = tk.Tk()
    app = CANMessageSender(root)
    root.mainloop()
