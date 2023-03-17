from typing import List
from typing import Optional
from sqlalchemy.orm import Mapped
from sqlalchemy.orm import mapped_column
from sqlalchemy.orm import relationship
from Base import Base
from sqlalchemy import String
from Reading import Reading

class Sensor(Base):
    __tablename__ = "sensor"
    id: Mapped[int] = mapped_column(primary_key=True)
    name: Mapped[str] = mapped_column(String(30))

    # readings: Mapped[List[Reading]] = relationship(back_populates="sensor")

    def __repr__(self) -> str:
        return "A Sensor"
        # return f"Sensor (id={self.id!r}, name={self.name!r}, readings={self.readings.count()})"
