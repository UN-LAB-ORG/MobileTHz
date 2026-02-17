// QTextEditStreamBuf.h
#ifndef QTEXTEDITSTREAMBUF_H
#define QTEXTEDITSTREAMBUF_H

#include <QMetaObject>
#include <QTextEdit>
#include <iostream>
#include <streambuf>
#include <string>

class QTextEditStreamBuf : public std::streambuf
{
public:
    explicit QTextEditStreamBuf(QTextEdit *textEdit, QObject *parent = nullptr)
        : m_textEdit(textEdit), m_parent(parent)
    {
    }

protected:
    // Called when the buffer is full or needs flushing (e.g., std::endl)
    int_type overflow(int_type c = traits_type::eof()) override
    {
        if (c != traits_type::eof())
        {
            m_buffer += traits_type::to_char_type(c);
        }
        // Flush buffer when a newline is encountered
        if (c == traits_type::eof() || c == '\n')
        {
            flushBuffer();
        }
        return c;
    }

    // Called to synchronize the buffer (explicit flush)
    int sync() override
    {
        flushBuffer();
        return 0; // Success
    }

private:
    void flushBuffer()
    {
        if (!m_buffer.empty())
        {
            QString textToInsert = QString::fromStdString(m_buffer);
            m_buffer.clear(); // Clear the internal buffer before the invokeMethod

            // Use invokeMethod to ensure thread safety when interacting with the GUI element
            QMetaObject::invokeMethod(
                m_textEdit,
                [this, textToInsert]()
                {
                    // Ensure operations happen on the GUI thread
                    m_textEdit->moveCursor(QTextCursor::End);
                    m_textEdit->insertPlainText(textToInsert);
                },
                Qt::QueuedConnection);
        }
    }

    QTextEdit *m_textEdit;
    QObject *m_parent;
    std::string m_buffer;
};

#endif // QTEXTEDITSTREAMBUF_H
