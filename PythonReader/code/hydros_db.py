import sqlalchemy
from sqlalchemy import text
from sqlalchemy.orm import Session
from sqlalchemy import Table, Column, Integer, String, MetaData
from Tables.Base import Base

class HydrosDB:

    def __init__(self, dbUrl):
        self.engine = sqlalchemy.create_engine(dbUrl, echo=True)
        self.metadata = MetaData()

    def goodbye_world(self):
        with self.engine.connect() as conn:
            conn = self.engine.connect()
            result = conn.execute(text("select 'Goodbye World'"))
            print(result.all())

    def hello_world(self):
        with Session(self.engine) as session:
            result = session.execute(text("select 'Hello World!'"))
            print(result.all())

    def create_table(self):
        user_table = Table(
            "user_account",
            self.metadata,
            Column("id", Integer, primary_key=True),
            Column("name", String(30)),
            Column("fullname", String),
        )
        self.metadata.create_all(self.engine)

    def create_tables(self):
        Base.metadata.create_all(self.engine)
        print("TABLES\n")
        self.metadata.reflect(self.engine)



if __name__ == '__main__':
    print(sqlalchemy.__version__)
    db = HydrosDB("sqlite+pysqlite:///:memory:")
    # db.hello_world()
    # db.goodbye_world()
    # db.create_table()
    db.create_tables()