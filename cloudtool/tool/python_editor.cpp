#include "python_editor.h"
#include "python/python_manager.h"
#include "python/python_bridge.h"
#include "python/python_worker.h"

#include <QVBoxLayout>
#include <QKeyEvent>
#include <QAction>
#include <QMainWindow>
#include <QFileDialog>
#include <QMessageBox>
#include <QTextStream>
#include <QFile>
#include <QFont>
#include <QShortcut>
#include <QScrollBar>
#include <QRegularExpression>
#include <QApplication>

namespace ct
{

// ================================================================
// PythonSyntaxHighlighter
// ================================================================

PythonSyntaxHighlighter::PythonSyntaxHighlighter(QTextDocument* parent)
    : QSyntaxHighlighter(parent)
{
    QTextCharFormat keyword_fmt;
    keyword_fmt.setForeground(QColor("#569cd6"));
    keyword_fmt.setFontWeight(QFont::Bold);

    QTextCharFormat builtin_fmt;
    builtin_fmt.setForeground(QColor("#6a9955"));
    builtin_fmt.setFontWeight(QFont::Bold);

    QTextCharFormat string_fmt;
    string_fmt.setForeground(QColor("#ce9178"));

    QTextCharFormat comment_fmt;
    comment_fmt.setForeground(QColor("#6a9955"));
    comment_fmt.setFontItalic(true);

    QTextCharFormat number_fmt;
    number_fmt.setForeground(QColor("#b5cea8"));

    QTextCharFormat decorator_fmt;
    decorator_fmt.setForeground(QColor("#dcdcaa"));
    decorator_fmt.setFontWeight(QFont::Bold);

    QTextCharFormat self_fmt;
    self_fmt.setForeground(QColor("#4ec9b0"));
    self_fmt.setFontWeight(QFont::Bold);

    QStringList keywords = {
        "def", "class", "if", "elif", "else", "for", "while", "return",
        "import", "from", "as", "try", "except", "finally", "with",
        "lambda", "yield", "pass", "break", "continue", "raise",
        "global", "nonlocal", "assert", "del", "and", "or", "not",
        "in", "is", "True", "False", "None"
    };
    for (const auto& kw : keywords) {
        Rule r;
        r.pattern = QRegularExpression(QString("\\b%1\\b").arg(kw));
        r.format = keyword_fmt;
        m_rules.append(r);
    }

    QStringList builtins = {
        "print", "len", "range", "int", "float", "str", "list", "dict",
        "tuple", "set", "type", "isinstance", "enumerate", "zip", "map",
        "filter", "sorted", "reversed", "open", "super", "property",
        "staticmethod", "classmethod"
    };
    for (const auto& b : builtins) {
        Rule r;
        r.pattern = QRegularExpression(QString("\\b%1\\b").arg(b));
        r.format = builtin_fmt;
        m_rules.append(r);
    }

    Rule self_rule;
    self_rule.pattern = QRegularExpression("\\b(self|cls)\\b");
    self_rule.format = self_fmt;
    m_rules.append(self_rule);

    Rule deco_rule;
    deco_rule.pattern = QRegularExpression("@\\w+");
    deco_rule.format = decorator_fmt;
    m_rules.append(deco_rule);

    Rule num_rule;
    num_rule.pattern = QRegularExpression("\\b[0-9]+\\.?[0-9]*\\b");
    num_rule.format = number_fmt;
    m_rules.append(num_rule);

    Rule str1_rule;
    str1_rule.pattern = QRegularExpression(R"('(?:[^'\\]|\\.)*')");
    str1_rule.format = string_fmt;
    m_rules.append(str1_rule);

    Rule str2_rule;
    str2_rule.pattern = QRegularExpression(R"("(?:[^"\\]|\\.)*")");
    str2_rule.format = string_fmt;
    m_rules.append(str2_rule);

    Rule comment_rule;
    comment_rule.pattern = QRegularExpression("#[^\n]*");
    comment_rule.format = comment_fmt;
    m_rules.append(comment_rule);
}

void PythonSyntaxHighlighter::highlightBlock(const QString& text)
{
    for (const auto& rule : m_rules) {
        auto it = rule.pattern.globalMatch(text);
        while (it.hasNext()) {
            auto match = it.next();
            setFormat(match.capturedStart(), match.capturedLength(), rule.format);
        }
    }
}

// ================================================================
// PythonEditor
// ================================================================

PythonEditor::PythonEditor(QWidget* parent)
    : QWidget(parent)
{
    // 不透明背景，防止底层 CloudView 透出
    setAttribute(Qt::WA_StyledBackground);
    setAutoFillBackground(true);
    QPalette pal = palette();
    pal.setColor(QPalette::Window, pal.color(QPalette::Base));
    setPalette(pal);

    // 右侧边框，与 CloudView 的白色边界条一致
    setStyleSheet("PythonEditor { border-right: 1px solid palette(mid); }");

    auto* main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(0, 0, 0, 0);
    main_layout->setSpacing(0);

    // === Toolbar — 使用系统主题色 ===
    m_toolbar = new QToolBar(this);
    m_toolbar->setIconSize(QSize(20, 20));
    m_toolbar->setMovable(false);
    m_toolbar->setToolButtonStyle(Qt::ToolButtonIconOnly);
    // 不强制颜色，跟随系统/应用主题
    m_toolbar->setStyleSheet(
        "QToolBar { background: palette(window); border-bottom: 1px solid palette(mid); padding: 2px 4px; }"
        "QToolBar QToolButton { padding: 3px; }");

    auto* action_new    = m_toolbar->addAction(QIcon(":/res/icon/document-new.svg"), "New (Ctrl+N)");
    auto* action_open   = m_toolbar->addAction(QIcon(":/res/icon/document-open.svg"), "Open (Ctrl+O)");
    auto* action_save   = m_toolbar->addAction(QIcon(":/res/icon/document-save.svg"), "Save (Ctrl+S)");
    auto* action_saveas = m_toolbar->addAction(QIcon(":/res/icon/document-save-as.svg"), "Save As");

    m_toolbar->addSeparator();

    m_action_run  = m_toolbar->addAction(QIcon(":/res/icon/start.svg"), "Run (F5)");
    m_action_stop = m_toolbar->addAction(QIcon(":/res/icon/stop.svg"), "Stop (Shift+F5)");
    m_action_stop->setEnabled(false);

    m_toolbar->addSeparator();

    auto* spacer = new QWidget(m_toolbar);
    spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    m_toolbar->addWidget(spacer);

    auto* action_close = m_toolbar->addAction(QIcon(":/res/icon/close.svg"), "Close Editor");

    connect(action_new,    &QAction::triggered, this, &PythonEditor::onNew);
    connect(action_open,   &QAction::triggered, this, &PythonEditor::onOpen);
    connect(action_save,   &QAction::triggered, this, &PythonEditor::onSave);
    connect(action_saveas, &QAction::triggered, this, &PythonEditor::onSaveAs);
    connect(m_action_run,  &QAction::triggered, this, &PythonEditor::onRun);
    connect(m_action_stop, &QAction::triggered, this, &PythonEditor::onStop);
    connect(action_close,  &QAction::triggered, this, &PythonEditor::hideEditor);

    main_layout->addWidget(m_toolbar);

    // === Tab Widget ===
    m_tabs = new QTabWidget(this);
    m_tabs->setTabsClosable(true);
    m_tabs->setMovable(true);
    m_tabs->setDocumentMode(true);

    connect(m_tabs, &QTabWidget::currentChanged,
            this, &PythonEditor::onTabChanged);
    connect(m_tabs, &QTabWidget::tabCloseRequested,
            this, &PythonEditor::onTabCloseRequested);

    main_layout->addWidget(m_tabs, 1);

    // === Status bar ===
    m_status = new QLabel("Ready", this);
    m_status->setStyleSheet(
        "QLabel { color: palette(text); background: palette(window); "
        "padding: 2px 6px; font-size: 10px; border-top: 1px solid palette(mid); }");
    main_layout->addWidget(m_status);

    // === 快捷键 ===
    new QShortcut(QKeySequence("F5"), this, SLOT(onRun()));
    new QShortcut(QKeySequence("Shift+F5"), this, SLOT(onStop()));
    new QShortcut(QKeySequence("Ctrl+N"), this, SLOT(onNew()));
    new QShortcut(QKeySequence("Ctrl+O"), this, SLOT(onOpen()));
    new QShortcut(QKeySequence("Ctrl+S"), this, SLOT(onSave()));
    new QShortcut(QKeySequence("Ctrl+W"), this, [this]() {
        onTabCloseRequested(m_tabs->currentIndex());
    });

    // === Worker 信号 ===
    auto* worker = ct::PythonManager::instance().worker();
    if (worker) {
        connect(worker, &PythonWorker::scriptStarted,
                this, &PythonEditor::onScriptStarted);
        connect(worker, &PythonWorker::scriptFinished,
                this, &PythonEditor::onScriptFinished);
    }

    // 初始隐藏
    hide();

    // 创建初始选项卡
    onNew();
}

EditorTab PythonEditor::createTab(const QString& title)
{
    EditorTab tab;
    tab.editor = new QPlainTextEdit(this);
    tab.editor->setFont(QFont("Consolas", 10));
    tab.editor->setLineWrapMode(QPlainTextEdit::NoWrap);
    tab.editor->setTabStopWidth(40);

    tab.highlighter = new PythonSyntaxHighlighter(tab.editor->document());

    connect(tab.editor, &QPlainTextEdit::modificationChanged, this,
            [this, idx = m_tab_list.size()](bool mod) {
        if (idx < m_tab_list.size()) {
            m_tab_list[idx].modified = mod;
            updateTabTitle(idx);
        }
    });

    tab.filepath.clear();
    tab.modified = false;
    m_tab_list.append(tab);

    int idx = m_tabs->addTab(tab.editor, title);
    m_tabs->setCurrentIndex(idx);

    return tab;
}

void PythonEditor::closeTab(int index)
{
    if (index < 0 || index >= m_tab_list.size()) return;

    auto& tab = m_tab_list[index];
    if (tab.modified) {
        auto ret = QMessageBox::question(this, "Unsaved Changes",
            QString("'%1' has unsaved changes. Save before closing?")
                .arg(tab.filepath.isEmpty() ? "Untitled" : QFileInfo(tab.filepath).fileName()),
            QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
        if (ret == QMessageBox::Cancel) return;
        if (ret == QMessageBox::Save) {
            m_tabs->setCurrentIndex(index);
            onSave();
        }
    }

    m_tabs->removeTab(index);
    delete tab.editor;
    m_tab_list.removeAt(index);

    if (m_tab_list.isEmpty()) {
        onNew();
    }
}

void PythonEditor::updateTabTitle(int index)
{
    if (index < 0 || index >= m_tab_list.size()) return;
    auto& tab = m_tab_list[index];
    QString title = tab.filepath.isEmpty()
        ? "Untitled"
        : QFileInfo(tab.filepath).fileName();
    if (tab.modified) title += " *";
    m_tabs->setTabText(index, title);
}

void PythonEditor::updateStatus()
{
    int idx = m_tabs->currentIndex();
    if (idx >= 0 && idx < m_tab_list.size()) {
        auto& tab = m_tab_list[idx];
        QString info;
        if (tab.filepath.isEmpty())
            info = "Untitled";
        else
            info = tab.filepath;
        if (m_busy)
            info += "  |  Running...";
        else
            info += "  |  Ready";
        m_status->setText(info);
    }
}

void PythonEditor::showEditor()
{
    QWidget* p = parentWidget();
    if (p) {
        // 右侧留 2px 间距，不覆盖 CloudView 与窗口边界的间隙
        QRect r = p->rect();
        r.setRight(r.right() - 2);
        setGeometry(r);
        p->installEventFilter(this);
        raise();
    }
    show();
    setFocus();

    // 同步菜单按钮状态
    if (auto* mw = qobject_cast<QMainWindow*>(window())) {
        if (auto* action = mw->findChild<QAction*>("actionPythonEditor")) {
            action->setChecked(true);
        }
    }
}

void PythonEditor::hideEditor()
{
    hide();

    // 同步菜单按钮状态
    if (auto* mw = qobject_cast<QMainWindow*>(window())) {
        if (auto* action = mw->findChild<QAction*>("actionPythonEditor")) {
            action->setChecked(false);
        }
    }
}

// === Slots ===

void PythonEditor::onRun()
{
    int idx = m_tabs->currentIndex();
    if (idx < 0 || idx >= m_tab_list.size()) return;

    auto& tab = m_tab_list[idx];
    QString code = tab.editor->toPlainText();
    if (code.trimmed().isEmpty()) return;

    auto* worker = ct::PythonManager::instance().worker();
    if (!worker || worker->isBusy()) return;

    worker->execScript(code,
        tab.filepath.isEmpty() ? "<editor>" : tab.filepath);
}

void PythonEditor::onStop()
{
    auto* worker = ct::PythonManager::instance().worker();
    if (worker && worker->isBusy()) {
        worker->cancel();
    }
}

void PythonEditor::onNew()
{
    createTab("Untitled");
}

void PythonEditor::onOpen()
{
    QString path = QFileDialog::getOpenFileName(
        this, "Open Python Script", QString(),
        "Python Files (*.py);;All Files (*)");
    if (path.isEmpty()) return;

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "Error", "Cannot open file: " + path);
        return;
    }

    QTextStream in(&file);
    QString content = in.readAll();
    file.close();

    auto tab = createTab(QFileInfo(path).fileName());
    tab.editor->setPlainText(content);
    tab.filepath = path;
    tab.modified = false;

    int idx = m_tab_list.size() - 1;
    updateTabTitle(idx);
    updateStatus();
}

void PythonEditor::onSave()
{
    int idx = m_tabs->currentIndex();
    if (idx < 0 || idx >= m_tab_list.size()) return;

    auto& tab = m_tab_list[idx];
    if (tab.filepath.isEmpty()) {
        onSaveAs();
        return;
    }

    QFile file(tab.filepath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "Error", "Cannot save file: " + tab.filepath);
        return;
    }

    QTextStream out(&file);
    out << tab.editor->toPlainText();
    file.close();

    tab.modified = false;
    tab.editor->document()->setModified(false);
    updateTabTitle(idx);
    updateStatus();
}

void PythonEditor::onSaveAs()
{
    int idx = m_tabs->currentIndex();
    if (idx < 0 || idx >= m_tab_list.size()) return;

    auto& tab = m_tab_list[idx];
    QString path = QFileDialog::getSaveFileName(
        this, "Save Python Script", QString(),
        "Python Files (*.py);;All Files (*)");
    if (path.isEmpty()) return;

    tab.filepath = path;
    onSave();
}

void PythonEditor::onTabChanged(int index)
{
    updateStatus();
}

void PythonEditor::onTabCloseRequested(int index)
{
    closeTab(index);
}

void PythonEditor::onScriptStarted()
{
    m_busy = true;
    m_action_run->setEnabled(false);
    m_action_stop->setEnabled(true);
    updateStatus();
}

void PythonEditor::onScriptFinished(bool ok, QString error)
{
    m_busy = false;
    m_action_run->setEnabled(true);
    m_action_stop->setEnabled(false);
    updateStatus();
}

void PythonEditor::keyPressEvent(QKeyEvent* event)
{
    if (event->key() == Qt::Key_Escape) {
        hideEditor();
        return;
    }
    QWidget::keyPressEvent(event);
}

bool PythonEditor::eventFilter(QObject* watched, QEvent* event)
{
    // 父 widget resize 时同步更新浮层大小
    if (watched == parentWidget() && event->type() == QEvent::Resize) {
        if (isVisible()) {
            QRect r = parentWidget()->rect();
            r.setRight(r.right() - 2);
            setGeometry(r);
        }
    }
    return QWidget::eventFilter(watched, event);
}

} // namespace ct
