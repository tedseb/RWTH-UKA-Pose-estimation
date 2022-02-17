class SMResponse:
    _CUSTOM_PAYLOAD_FIELDS = set({
        "error"
        })

    def __init__(self, response_code = 500, status_code = 8, payload = dict(), request_requiered = True):
        self.response_code = response_code
        self.status_code = status_code
        self.payload = payload
        self.request_requiered = request_requiered

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return f"SMResponse({self.response_code}, {self.status_code}, {self.payload})"

    def __eq__(self, other):
        if not isinstance(other, SMResponse):
            return False

        if self.response_code != other.response_code:
            return False
        if self.status_code != other.status_code:
            return False

        for key in other.payload.keys():
            if key in self._CUSTOM_PAYLOAD_FIELDS:
                continue
            if key not in self.payload:
                return False

        for key, item in self.payload.items():
            if key in self._CUSTOM_PAYLOAD_FIELDS:
                continue
            if key not in other.payload:
                return False

            if item != other.payload[key]:
                return False
        return True