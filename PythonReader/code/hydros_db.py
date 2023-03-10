import sqlalchemy
from sqlalchemy import text
from sqlalchemy.orm import Session

class HydrosDB:

    def __init__(self, dbUrl):
        self.engine = sqlalchemy.create_engine(dbUrl, echo=True)

    def goodbye_world(self):
        with self.engine.connect() as conn:
            conn = self.engine.connect()
            result = conn.execute(text("select 'Goodbye World'"))
            print(result.all())

    def hello_world(self):
        with Session(self.engine) as session:
            result = session.execute(text("select 'Hello World!'"))
            print(result.all())

if __name__ == '__main__':
    print(sqlalchemy.__version__)
    db = HydrosDB("sqlite+pysqlite:///:memory:")
    db.hello_world()
    db.goodbye_world()