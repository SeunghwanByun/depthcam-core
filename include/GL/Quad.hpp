#pragma once

#include <GL/glew.h>

namespace Youth::GL
{
    class Quad
    {
    public:
        Quad() = default;
        ~Quad() { cleanup(); };

        bool init();
        void render();
        void cleanup();

    private:
        GLuint vao_ = 0, vbo_ = 0, ebo_ = 0;
    };
}