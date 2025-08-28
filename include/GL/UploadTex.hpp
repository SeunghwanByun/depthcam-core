#pragma once
#include <GL/glew.h>
#include <cstdint>

namespace Youth::GL
{
    class UploadTex
    {
    public:
        ~UploadTex() { destroy(); }
        bool ensure(int w, int h)
        {
            if (tex_ && (w == w_ && h == h_))
                return true;
            destroy();
            w_ = w;
            h_ = h;
            glGenTextures(1, &tex_);
            glBindTexture(GL_TEXTURE_2D, tex_);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, w_, h_, 0, GL_RED, GL_UNSIGNED_BYTE, nullptr);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glBindTexture(GL_TEXTURE_2D, 0);
            return true;
        }
        void update(const uint8_t *data)
        {
            glBindTexture(GL_TEXTURE_2D, tex_);
            glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w_, h_, GL_RED, GL_UNSIGNED_BYTE, data);
            glBindTexture(GL_TEXTURE_2D, 0);
        }

        GLuint id() const { return tex_; }
        int width() const { return w_; }
        int height() const { return h_; }

        void destroy()
        {
            if (tex_)
            {
                glDeleteTextures(1, &tex_);
                tex_ = 0;
            }
            w_ = h_ = 0;
        }

    private:
        GLuint tex_ = 0;
        int w_ = 0, h_ = 0;
    };
}