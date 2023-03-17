from typing import List
from typing import Optional
from sqlalchemy.orm import Mapped
from sqlalchemy.orm import mapped_column
from sqlalchemy.orm import relationship
from Base import Base
from sqlalchemy import ForeignKey, Numeric, String

from Sensor import Sensor

class Reading(object):
    __tablename__ = "reading"
    id: Mapped[int] = mapped_column(primary_key=True)
    sensor_id = mapped_column(ForeignKey("sensor.id"))
    value: Mapped[Numeric] = mapped_column(Numeric)

    sensor: Mapped[Sensor] = relationship(back_populates="readings")

    def __repr__(self) -> str:
        return f"Reading (id={self.id!r}, sensor={self.sensor.name!r}, )"
