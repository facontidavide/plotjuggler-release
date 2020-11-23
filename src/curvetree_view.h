#ifndef CURVETREE_VIEW_H
#define CURVETREE_VIEW_H

#include "curvelist_view.h"
#include <QTreeWidget>
#include <functional>

class CurveTreeView : public QTreeWidget, public CurvesView
{
public:
  CurveTreeView(CurveListPanel* parent);

  void clear() override
  {
    QTreeWidget::clear();
    _leaf_count = 0;
    _hidden_count = 0;
  }

  void addItem(const QString& item_name) override;

  void refreshColumns() override;

  std::vector<std::string> getSelectedNames() override;

  void refreshFontSize() override;

  bool applyVisibilityFilter(const QString& filter_string) override;

  bool eventFilter(QObject* object, QEvent* event) override
  {
    bool ret = CurvesView::eventFilterBase(object, event);
    if (!ret)
    {
      return QWidget::eventFilter(object, event);
    }
    else
    {
      return true;
    }
  }

  void removeCurve(const QString& name) override;

  std::pair<int, int> hiddenItemsCount() override
  {
    return { _hidden_count, _leaf_count };
  }

  void setViewResizeEnabled(bool) override
  { }

  virtual void hideValuesColumn(bool hide) override;

  void treeVisitor(std::function<void(QTreeWidgetItem*)> visitor);

private:

  void expandChildren(QTreeWidgetItem *item);

  int _hidden_count = 0;
  int _leaf_count = 0;
};

#endif  // CURVETREE_VIEW_H
