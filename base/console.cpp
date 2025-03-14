#include "base/console.h"

#include <QDateTime>

namespace ct
{
    void Console::print(log_level level, const QString &message)
    {
        // currentDateTime()是QDateTime类的一个静态函数，它返回一个QDateTime对象，该对象表示当前的日期和时间。
        // toString()是QDateTime类的一个成员函数，用于将日期和时间转换为字符串。
        QString currentTime = QDateTime::currentDateTime().toString("hh:mm:ss");
        QString level_color;
        switch (level)
        {
            case log_level::LOG_INFO:
                // 使用HTML标签来设置文本的颜色，并将其与消息文本结合以生成富文本字符串
                // <font color='black'>是一个HTML标签，用于设置文本的颜色
                // </font>是font标签的结束标签，用于关闭之前开始的font标签。
                level_color = "<font color='black'>" + message + "</font>";
                break;
            case log_level::LOG_WARNING:
                level_color = "<font color='#CFBF17'>" + message + "</font>";
                break;
            case log_level::LOG_ERROR:
                level_color = "<font color='red'>" + message + "</font>";
                break;
        }
        // 将当前时间和带有颜色的日志级别信息拼接在一起，并将结果添加到文本框末尾
        append("[" + currentTime + "]:" + level_color);
        // 将文本控件的光标移动到文档的末尾,moveCursor是QTextEdit或QPlainTextEdit的成员函数，用于设置光标的位置。
        moveCursor(QTextCursor::End);
    }
}
