"""
SPARTN GNSS Receiver configuration dialog

Created on 26 Jan 2023

FYI CFG_NAVSPG_CONSTR_DGNSSTO governs how quickly rcvr drops out of RTK mode
after loss of correction signal - default is 60 seconds but this may need
to be increased if the correction source is intermittent.

:author: semuadmin
:copyright: SEMU Consulting © 2023
:license: BSD 3-Clause
"""
# pylint: disable=invalid-name, too-many-locals, too-many-instance-attributes

from datetime import datetime, timedelta
from tkinter import (
    ttk,
    Frame,
    Button,
    Checkbutton,
    Radiobutton,
    Label,
    Entry,
    StringVar,
    IntVar,
    E,
    W,
    NORMAL,
)
from PIL import ImageTk, Image
from pyubx2 import (
    UBXMessage,
    val2bytes,
    SET,
    POLL,
    U1,
    U2,
    U4,
    TXN_NONE,
    SET_LAYER_RAM,
)
from pygpsclient.globals import (
    ICON_SEND,
    ICON_CONFIRMED,
    ICON_PENDING,
    ICON_WARNING,
    ICON_DISCONN,
    ICON_BLANK,
    ENTCOL,
    CONNECTED,
    SPARTN_SOURCE_LB,
    SPARTN_SOURCE_IP,
    SPARTN_GNSS,
    SPARTN_KEYLEN,
    RXMMSG,
)
from pygpsclient.strings import (
    LBLSPTNCURR,
    LBLSPTNNEXT,
    LBLSPTNKEY,
    LBLSPTNDAT,
    LBLSPTNUPLOAD,
    LBLSPTNFP,
    LBLSPTNNMEA,
    NOTCONN,
    NULLSEND,
    LBLSPARTNGN,
    CONFIGOK,
    CONFIGBAD,
)
from pygpsclient.helpers import (
    valid_entry,
    date2wnotow,
    parse_rxmspartnkey,
    VALHEX,
    VALDMY,
    VALLEN,
)

U2MAX = 2e16 - 1
U8MAX = 2e64 - 1
BAUDRATE = 38400
RESERVED0 = b"\x00\x00"
RESERVED1 = b"\x00"
CFGSET = "CFG-VALGET/SET"
CFGPOLL = "CFG-VALGET"
INPORTS = ("I2C", "UART1", "UART2", "USB", "SPI")


class SPARTNGNSSDialog(Frame):
    """,
    SPARTNConfigDialog class.
    """

    def __init__(
        self, app, container, *args, **kwargs
    ):  # pylint: disable=unused-argument
        """
        Constructor.

        :param Frame app: reference to main tkinter application
        :param Frame container: reference to container frame
        :param args: optional args to pass to parent class (not currently used)
        :param kwargs: optional kwargs to pass to parent class (not currently used)
        """

        self.__app = app  # Reference to main application class
        self.__master = self.__app.appmaster  # Reference to root class (Tk)
        self.__container = container  # container frame

        Frame.__init__(self, self.__container.container, *args, **kwargs)

        self._img_blank = ImageTk.PhotoImage(Image.open(ICON_BLANK))
        self._img_pending = ImageTk.PhotoImage(Image.open(ICON_PENDING))
        self._img_confirmed = ImageTk.PhotoImage(Image.open(ICON_CONFIRMED))
        self._img_warn = ImageTk.PhotoImage(Image.open(ICON_WARNING))
        self._img_send = ImageTk.PhotoImage(Image.open(ICON_SEND))
        self._img_disconn = ImageTk.PhotoImage(Image.open(ICON_DISCONN))
        self._status = StringVar()
        self._spartn_source = IntVar()
        self._status_cfgmsg = StringVar()
        self._spartn_key1 = StringVar()
        self._spartn_valdate1 = StringVar()
        self._spartn_key2 = StringVar()
        self._spartn_valdate2 = StringVar()
        self._upload_keys = IntVar()
        self._send_f9p_config = IntVar()
        self._disable_nmea = IntVar()
        self._ports = INPORTS

        self._body()
        self._do_layout()
        self._reset()

    def _body(self):
        """
        Set up frame and widgets.
        """
        # pylint: disable=unnecessary-lambda

        # GNSS Receiver (F9P) configuration widgets
        self._lbl_gnss = Label(self, text=LBLSPARTNGN)
        self._lbl_curr = Label(self, text=LBLSPTNCURR)
        self._lbl_key1 = Label(self, text=LBLSPTNKEY)
        self._ent_key1 = Entry(
            self,
            textvariable=self._spartn_key1,
            bg=ENTCOL,
            state=NORMAL,
            relief="sunken",
            width=32,
        )
        self._lbl_valdate1 = Label(self, text=LBLSPTNDAT)
        self._ent_valdate1 = Entry(
            self,
            textvariable=self._spartn_valdate1,
            bg=ENTCOL,
            state=NORMAL,
            relief="sunken",
            width=8,
        )
        self._lbl_next = Label(self, text=LBLSPTNNEXT)
        self._lbl_key2 = Label(self, text=LBLSPTNKEY)
        self._ent_key2 = Entry(
            self,
            textvariable=self._spartn_key2,
            bg=ENTCOL,
            state=NORMAL,
            relief="sunken",
            width=32,
        )
        self._lbl_valdate2 = Label(self, text=LBLSPTNDAT)
        self._ent_valdate2 = Entry(
            self,
            textvariable=self._spartn_valdate2,
            bg=ENTCOL,
            state=NORMAL,
            relief="sunken",
            width=8,
        )
        self._rad_source0 = Radiobutton(
            self, text="IP", variable=self._spartn_source, value=SPARTN_SOURCE_IP
        )
        self._rad_source1 = Radiobutton(
            self,
            text="L-Band",
            variable=self._spartn_source,
            value=SPARTN_SOURCE_LB,
        )
        self._chk_upload_keys = Checkbutton(
            self,
            text=LBLSPTNUPLOAD,
            variable=self._upload_keys,
        )
        self._chk_send_config = Checkbutton(
            self,
            text=LBLSPTNFP,
            variable=self._send_f9p_config,
        )
        self._chk_disable_nmea = Checkbutton(
            self, text=LBLSPTNNMEA, variable=self._disable_nmea
        )
        self._lbl_send_command = Label(self, image=self._img_blank)
        self._btn_send_command = Button(
            self,
            image=self._img_send,
            width=45,
            fg="green",
            command=self._on_send_gnss_config,
            font=self.__app.font_md,
        )

    def _do_layout(self):
        """
        Position widgets in frame.
        """

        self._lbl_gnss.grid(column=0, row=0, columnspan=4, padx=3, pady=2, sticky=W)
        self._lbl_curr.grid(column=0, row=1, columnspan=4, padx=3, pady=2, sticky=W)
        self._lbl_key1.grid(column=0, row=2, padx=3, pady=2, sticky=W)
        self._ent_key1.grid(column=1, row=2, columnspan=3, padx=3, pady=2, sticky=W)
        self._lbl_valdate1.grid(column=0, row=3, padx=3, pady=2, sticky=W)
        self._ent_valdate1.grid(column=1, row=3, columnspan=3, padx=3, pady=2, sticky=W)
        ttk.Separator(self).grid(
            column=0, row=4, columnspan=4, padx=2, pady=3, sticky=(W, E)
        )
        self._lbl_next.grid(column=0, row=5, columnspan=3, padx=3, pady=2, sticky=W)
        self._lbl_key2.grid(column=0, row=6, padx=3, pady=2, sticky=W)
        self._ent_key2.grid(column=1, row=6, columnspan=3, padx=2, pady=2, sticky=W)
        self._lbl_valdate2.grid(column=0, row=7, padx=3, pady=2, sticky=W)
        self._ent_valdate2.grid(column=1, row=7, columnspan=3, padx=3, pady=2, sticky=W)
        ttk.Separator(self).grid(
            column=0, row=8, columnspan=4, padx=2, pady=3, sticky=(W, E)
        )
        self._rad_source0.grid(column=0, row=9, padx=3, pady=2, sticky=W)
        self._rad_source1.grid(column=1, row=9, columnspan=3, padx=3, pady=2, sticky=W)
        self._chk_upload_keys.grid(column=0, row=10, padx=3, pady=2, sticky=W)
        self._chk_send_config.grid(column=1, row=10, padx=3, pady=2, sticky=W)
        self._chk_disable_nmea.grid(
            column=2, row=10, columnspan=2, padx=3, pady=2, sticky=W
        )
        ttk.Separator(self).grid(
            column=0, row=11, columnspan=4, padx=2, pady=3, sticky=(W, E)
        )
        self._btn_send_command.grid(column=2, row=12, padx=3, pady=2, sticky=W)
        self._lbl_send_command.grid(column=3, row=12, padx=3, pady=2, sticky=W)

    def _reset(self):
        """
        Reset configuration widgets.
        """

        curr = datetime.now()
        self._set_date(self._spartn_valdate1, curr)
        self._set_date(self._spartn_valdate2, curr + timedelta(weeks=4))
        self._spartn_source.set(SPARTN_SOURCE_IP)
        self._upload_keys.set(1)
        self._send_f9p_config.set(1)
        self._disable_nmea.set(0)

    def _valid_gnss_settings(self) -> bool:
        """
        Validate GNSS receiver settings.

        :return: valid True/False
        :rtype: bool
        """

        valid = True
        # key must be valid hexadecimal and 16 bytes (32 chars) in length
        valid = (
            valid
            & valid_entry(self._ent_key1, VALHEX)
            & valid_entry(self._ent_key1, VALLEN, SPARTN_KEYLEN * 2, SPARTN_KEYLEN * 2)
        )
        # from date must be valid date in format YYYYMMDD
        valid = valid & valid_entry(self._ent_valdate1, VALDMY)
        valid = (
            valid
            & valid_entry(self._ent_key2, VALHEX)
            & valid_entry(self._ent_key2, VALLEN, SPARTN_KEYLEN * 2, SPARTN_KEYLEN * 2)
        )
        valid = valid & valid_entry(self._ent_valdate2, VALDMY)

        if not valid:
            self.__container.set_status("ERROR - invalid settings", "red")

        return valid

    def _format_cfggnss(self) -> UBXMessage:
        """
        Format UBX CFG-VALSET message to configure GNSS receiver.
        """

        cfgdata = []
        # select SPARTN source (L-band or IP)
        cfgdata.append(("CFG_SPARTN_USE_SOURCE", self._spartn_source.get()))
        # enable SPARTN and UBX on selected ports
        for port in self._ports:
            cfgdata.append((f"CFG_{port}INPROT_UBX", 1))
            cfgdata.append((f"CFG_{port}INPROT_RTCM3X", 1))
            cfgdata.append((f"CFG_{port}INPROT_SPARTN", 1))
            cfgdata.append((f"CFG_MSGOUT_UBX_RXM_COR_{port}", 1))

            # if NMEA disabled, remove it from out ports and enable
            # a couple of UBX NAV messages instead
            if self._disable_nmea.get():
                cfgdata.append((f"CFG_{port}OUTPROT_NMEA", 0))
                cfgdata.append((f"CFG_MSGOUT_UBX_NAV_PVT_{port}", 1))
                cfgdata.append((f"CFG_MSGOUT_UBX_NAV_SAT_{port}", 4))
            else:
                cfgdata.append((f"CFG_{port}OUTPROT_NMEA", 1))
                # cfgdata.append((f"CFG_MSGOUT_UBX_NAV_PVT_{port}", 0))
                # cfgdata.append((f"CFG_MSGOUT_UBX_NAV_SAT_{port}", 0))

        msg = UBXMessage.config_set(SET_LAYER_RAM, TXN_NONE, cfgdata)
        return msg

    def _format_rxmspartn(self) -> UBXMessage:
        """
        Format UBX RXM-SPARTN-KEY message.
        """

        version = val2bytes(1, U1)
        numKeys = val2bytes(2, U1)
        key1 = bytes.fromhex(self._spartn_key1.get())
        keylen1 = val2bytes(len(key1), U1)
        wno1, tow1 = date2wnotow(self._get_date(self._spartn_valdate1))
        wno1b = val2bytes(wno1, U2)
        tow1b = val2bytes(tow1, U4)
        key2 = bytes.fromhex(self._spartn_key1.get())
        keylen2 = val2bytes(len(key2), U1)
        wno2, tow2 = date2wnotow(self._get_date(self._spartn_valdate2))
        wno2b = val2bytes(wno2, U2)
        tow2b = val2bytes(tow2, U4)

        payload = (
            version
            + numKeys
            + RESERVED0
            + RESERVED1
            + keylen1
            + wno1b
            + tow1b
            + RESERVED1
            + keylen2
            + wno2b
            + tow2b
            + key1
            + key2
        )
        msg = UBXMessage("RXM", RXMMSG, SET, payload=payload)
        return msg

    def _on_send_gnss_config(self):
        """
        Send config to GNSS receiver (e.g. F9P).
        """

        if not self._valid_gnss_settings():
            return

        if self.__app.conn_status != CONNECTED:
            self.__container.set_status(NOTCONN, "red")
            # return

        if self._send_f9p_config.get():
            msgc = "config"
            msg = self._format_cfggnss()
            self._send_gnss_command(msg)
        else:
            msgc = ""
        if self._upload_keys.get():
            msgk = "keys"
            msg = self._format_rxmspartn()
            self._send_gnss_command(msg)
            # poll RXM-SPARTN-KEY to check keys have loaded
            msg = UBXMessage("RXM", RXMMSG, POLL)
            self._send_gnss_command(msg)
        else:
            msgk = ""
        if msgk == "" and msgc == "":
            self.__container.set_status(NULLSEND, "red")
            return
        if msgk != "" and msgc != "":
            msga = " and "
        else:
            msga = ""
        self.__container.set_status(
            f"{(msgk + msga + msgc).capitalize()} sent", "green"
        )
        self._lbl_send_command.config(image=self._img_pending)
        for msgid in ("RXM-SPARTNKEY", "CFG-VALGET", "ACK-ACK", "ACK-NAK"):
            self.__container.set_pending(msgid, SPARTN_GNSS)

    def _send_gnss_command(self, msg: UBXMessage):
        """
        Send command to GNSS receiver.
        """

        self.__app.gnss_outqueue.put(msg.serialize())

    def _set_date(self, val: StringVar, dat: datetime):
        """
        Set date string in entry field.
        """

        val.set(dat.strftime("%Y%m%d"))

    def _get_date(self, val: StringVar) -> datetime:
        """
        Get datetime from entry field.
        """

        dat = val.get()
        return datetime(int(dat[0:4]), int(dat[4:6]), int(dat[6:8]))

    def update_status(self, msg: UBXMessage):
        """
        Update pending confirmation status.
        :param UBXMessage msg: UBX config message
        """

        if msg.identity == RXMMSG:
            if msg.numKeys == 2:  # check both keys have been uploaded
                self._lbl_send_command.config(image=self._img_confirmed)
                keydata = parse_rxmspartnkey(msg)  # key1, date1, key2, date2
                self._spartn_key1.set(keydata[0][0])
                self._spartn_valdate1.set(keydata[0][1].strftime("%Y%m%d"))
                self._spartn_key2.set(keydata[1][0])
                self._spartn_valdate2.set(keydata[1][1].strftime("%Y%m%d"))
                self.__container.set_status(CONFIGOK.format(RXMMSG), "green")
            else:
                self._lbl_send_command.config(image=self._img_warn)
                self.__container.set_status(CONFIGBAD.format(RXMMSG), "red")
        elif msg.identity == "ACK-ACK":
            self._lbl_send_command.config(image=self._img_confirmed)
            self.__container.set_status(CONFIGOK.format(CFGSET), "green")
        elif msg.identity == "ACK-NAK":
            self._lbl_send_command.config(image=self._img_warn)
            self.__container.set_status(CONFIGBAD.format(CFGSET), "red")
        self.update_idletasks()
