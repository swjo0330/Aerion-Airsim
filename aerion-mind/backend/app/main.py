from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.api.routes import router
from app.core.config import settings


def create_app() -> FastAPI:
    app = FastAPI(
        title="AERION MIND Tactical Server",
        version="1.0.0",
        description="Strategic Planning Loop: intent parsing, planning graph, safety validation, and A2A dispatch to AERION Soma.",
    )

    app.add_middleware(
        CORSMiddleware,
        allow_origins=settings.cors_origins,
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    app.include_router(router, prefix="/api/v1")

    @app.get("/")
    async def root():
        return {"service": "AERION MIND", "status": "running", "docs": "/docs"}

    return app


app = create_app()
