#ifndef CLOUDTOOL2_PYTHON_EDITOR_H
#define CLOUDTOOL2_PYTHON_EDITOR_H

#include <QWidget>
#include <QPlainTextEdit>
#include <QTabWidget>
#include <QToolBar>
#include <QLabel>
#include <QSyntaxHighlighter>
#include <QRegularExpression>
#include <QList>
#include <QAction>

namespace ct
{

// ================================================================
// Python 语法高亮器
// ================================================================
class PythonSyntaxHighlighter : public QSyntaxHighlighter
{
    Q_OBJECT

public:
    explicit PythonSyntaxHighlighter(QTextDocument* parent = nullptr);

protected:
    void highlightBlock(const QString& text) override;

private:
    struct Rule {
        QRegularExpression pattern;
        QTextCharFormat format;
    };
    QList<Rule> m_rules;
};

// ================================================================
// 编辑器选项卡数据
// ================================================================
struct EditorTab {
    QPlainTextEdit* editor = nullptr;
    PythonSyntaxHighlighter* highlighter = nullptr;
    QString filepath;       // 空表示未保存
    bool modified = false;
};

// ================================================================
// PythonEditor 浮层（覆盖在 CloudView 之上）
// ================================================================
class PythonEditor : public QWidget
{
    Q_OBJECT

public:
    explicit PythonEditor(QWidget* parent = nullptr);

    void showEditor();
    void hideEditor();

private slots:
    void onRun();
    void onStop();
    void onNew();
    void onOpen();
    void onSave();
    void onSaveAs();
    void onTabChanged(int index);
    void onTabCloseRequested(int index);
    void onScriptStarted();
    void onScriptFinished(bool ok, QString error);

private:
    QToolBar*     m_toolbar;
    QTabWidget*   m_tabs;
    QLabel*       m_status;
    QList<EditorTab> m_tab_list;
    QAction*      m_action_run;
    QAction*      m_action_stop;
    bool          m_busy = false;

    EditorTab createTab(const QString& title = "Untitled");
    void closeTab(int index);
    void updateTabTitle(int index);
    void updateStatus();

    void keyPressEvent(QKeyEvent* event) override;
    bool eventFilter(QObject* watched, QEvent* event) override;
};

} // namespace ct

#endif // CLOUDTOOL2_PYTHON_EDITOR_H
