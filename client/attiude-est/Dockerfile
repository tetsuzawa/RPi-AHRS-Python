FROM python:3.7.5-buster

WORKDIR /usr/src/app

COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

#CMD [ "python3", "./main_client.py" ]
