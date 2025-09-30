FROM python:3.11-slim

WORKDIR /app

COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

ENV PROTO=tcp \
    CMD_HOST=host.docker.internal \
    CMD_PORT=5555 \
    TEL_HOST=0.0.0.0 \
    TEL_PORT=5600 \
    PYTHONUNBUFFERED=1

CMD ["python", "explore_client.py"]
