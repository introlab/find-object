// Taken from UtiLite library r186 [www.utilite.googlecode.com]

/*
*  utilite is a cross-platform library with
*  useful utilities for fast and small developing.
*  Copyright (C) 2010  Mathieu Labbe
*
*  utilite is free library: you can redistribute it and/or modify
*  it under the terms of the GNU Lesser General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  utilite is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU Lesser General Public License for more details.
*
*  You should have received a copy of the GNU Lesser General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef UPLOT_H_
#define UPLOT_H_

//#include "utilite/UtiLiteExp.h" // DLL export/import defines

#include <QFrame>
#include <QtCore/QList>
#include <QtCore/QMap>
#include <QtGui/QPen>
#include <QtGui/QBrush>
#include <QGraphicsEllipseItem>
#include <QtCore/QMutex>
#include <QLabel>
#include <QPushButton>
#include <QtCore/QTime>
#include <QtCore/QElapsedTimer>

class QGraphicsView;
class QGraphicsScene;
class QGraphicsItem;
class QFormLayout;

/**
 * UPlotItem is a QGraphicsEllipseItem and can be inherited to do custom behaviors
 * on an hoverEnterEvent() for example.
 */
class UPlotItem : public QGraphicsEllipseItem
{
public:
	/**
	 * Constructor 1.
	 */
	UPlotItem(qreal dataX, qreal dataY, qreal width=2);
	/**
	 * Constructor 2.
	 */
	UPlotItem(const QPointF & data, qreal width=2);
	virtual ~UPlotItem();

public:
	void setNextItem(UPlotItem * nextItem);
	void setPreviousItem(UPlotItem * previousItem);
	void setData(const QPointF & data);

	UPlotItem * nextItem() const {return _nextItem;}
	UPlotItem * previousItem() const {return _previousItem;};
	const QPointF & data() const {return _data;}

protected:
	virtual void hoverEnterEvent(QGraphicsSceneHoverEvent * event);
	virtual void hoverLeaveEvent(QGraphicsSceneHoverEvent * event);
	virtual void focusInEvent(QFocusEvent * event);
	virtual void focusOutEvent(QFocusEvent * event);
	virtual void keyReleaseEvent(QKeyEvent * keyEvent);

	virtual void showDescription(bool shown);

private:
	QPointF _data;
	QGraphicsTextItem * _text;
	UPlotItem * _previousItem;
	UPlotItem * _nextItem;
};

class UPlot;

/**
 * UPlotCurve is a curve used to hold data shown in a UPlot.
 */
class UPlotCurve : public QObject
{
	Q_OBJECT

public:
	/**
	 * Constructor 1
	 */
	UPlotCurve(const QString & name, QObject * parent = 0);
	/**
	 * Constructor 2
	 */
	UPlotCurve(const QString & name, const QVector<UPlotItem *> data, QObject * parent = 0);
	/**
	 * Constructor 3
	 */
	UPlotCurve(const QString & name, const QVector<float> & x, const QVector<float> & y, QObject * parent = 0);
	virtual ~UPlotCurve();

	/**
	 * Get pen.
	 */
	const QPen & pen() const {return _pen;}
	/**
	 * Get brush.
	 */
	const QBrush & brush() const {return _brush;}

	/**
	 * Set pen.
	 */
	void setPen(const QPen & pen);
	/**
	 * Set brush.
	 */
	void setBrush(const QBrush & brush);

	/**
	 * Get name.
	 */
	QString name() const {return _name;}
	/**
	 * Get the number of items in the curve (dot + line items).
	 */
	int itemsSize() const;
	QPointF getItemData(int index);
	bool isVisible() const {return _visible;}
	void setData(QVector<UPlotItem*> & data); // take the ownership
	void setData(const QVector<float> & x, const QVector<float> & y);
	void setData(const std::vector<float> & x, const std::vector<float> & y);
	void setData(const QVector<float> & y);
	void setData(const std::vector<float> & y);
	void getData(QVector<float> & x, QVector<float> & y) const; // only call in Qt MainThread
	void draw(QPainter * painter);

public Q_SLOTS:
	/**
	 *
	 * Clear curve's values.
	 */
	virtual void clear();
	/**
	 *
	 * Show or hide the curve.
	 */
    void setVisible(bool visible);
    /**
     *
	 * Set increment of the x values (when auto-increment is used).
	 */
    void setXIncrement(float increment);
    /**
     *
	 * Set starting x value (when auto-increment is used).
	 */
    void setXStart(float val);
    /**
     *
	 * Add a single value, using a custom UPlotItem.
	 */
	void addValue(UPlotItem * data); // take the ownership
	/**
	 *
	 * Add a single value y, x is auto-incremented by the increment set with setXIncrement().
	 * @see setXStart()
	 */
	void addValue(float y);
	/**
	 *
	 * Add a single value y at x.
	 */
	void addValue(float x, float y);
	/**
	 *
	 * For convenience...
	 * Add a single value y, x is auto-incremented by the increment set with setXIncrement().
	 * @see setXStart()
	 */
	void addValue(const QString & y);
	/**
	 *
	 * For convenience...
	 * Add multiple values, using custom UPlotItem.
	 */
	void addValues(QVector<UPlotItem *> & data); // take the ownership
	/**
	 *
	 * Add multiple values y at x. Vectors must have the same size.
	 */
	void addValues(const QVector<float> & xs, const QVector<float> & ys);
	/**
	 *
	 * Add multiple values y, x is auto-incremented by the increment set with setXIncrement().
	 * @see setXStart()
	 */
	void addValues(const QVector<float> & ys);
	void addValues(const QVector<int> & ys); // for convenience
	/**
	 *
	 * Add multiple values y, x is auto-incremented by the increment set with setXIncrement().
	 * @see setXStart()
	 */
	void addValues(const std::vector<float> & ys); // for convenience
	void addValues(const std::vector<int> & ys); // for convenience

Q_SIGNALS:
	/**
	 *
	 *  emitted when data is changed.
	 */
	void dataChanged(const UPlotCurve *);

protected:
	friend class UPlot;
	void attach(UPlot * plot);
	void detach(UPlot * plot);
	void updateMinMax();
	const QVector<float> & getMinMax() const {return _minMax;}
	int removeItem(int index);
	void _addValue(UPlotItem * data);;
	virtual bool isMinMaxValid() const {return _minMax.size();}
	virtual void update(float scaleX, float scaleY, float offsetX, float offsetY, float xDir, float yDir, bool allDataKept);
	QList<QGraphicsItem *> _items;
	UPlot * _plot;

private:
	void removeItem(UPlotItem * item);

private:
	QString _name;
	QPen _pen;
	QBrush _brush;
	float _xIncrement;
	float _xStart;
	bool _visible;
	bool _valuesShown;
	QVector<float> _minMax; // minX, maxX, minY, maxY
};


/**
 * A special UPlotCurve that shows as a line at the specified value, spanning all the UPlot.
 */
class UPlotCurveThreshold : public UPlotCurve
{
	Q_OBJECT

public:
	/**
	 * Constructor.
	 */
	UPlotCurveThreshold(const QString & name, float thesholdValue, Qt::Orientation orientation = Qt::Horizontal, QObject * parent = 0);
	virtual ~UPlotCurveThreshold();

public Q_SLOTS:
	/**
	 * Set threshold value.
	 */
	void setThreshold(float threshold);
	/**
	 * Set orientation (Qt::Horizontal or Qt::Vertical).
	 */
	void setOrientation(Qt::Orientation orientation);

protected:
	friend class UPlot;
	virtual void update(float scaleX, float scaleY, float offsetX, float offsetY, float xDir, float yDir, bool allDataKept);
	virtual bool isMinMaxValid() const {return false;}

private:
	Qt::Orientation _orientation;
};

/**
 * The UPlot axis object.
 */
class UPlotAxis : public QWidget
{
public:
	/**
	 * Constructor.
	 */
	UPlotAxis(Qt::Orientation orientation = Qt::Horizontal, float min=0, float max=1, QWidget * parent = 0);
	virtual ~UPlotAxis();

public:
	/**
	 * Set axis minimum and maximum values, compute the resulting
	 * intervals depending on the size of the axis.
	 */
	void setAxis(float & min, float & max);
	/**
	 * Size of the border between the first line and the beginning of the widget.
	 */
	int border() const {return _border;}
	/**
	 * Interval step value.
	 */
	int step() const {return _step;}
	/**
	 * Number of intervals.
	 */
	int count() const {return _count;}
	/**
	 * Reverse the axis (for vertical :bottom->up, for horizontal :right->left)
	 */
	void setReversed(bool reversed); // Vertical :bottom->up, horizontal :right->left

protected:
	virtual void paintEvent(QPaintEvent * event);

private:
	Qt::Orientation _orientation;
	float _min;
	float _max;
	int _count;
	int _step;
	bool _reversed;
	int _gradMaxDigits;
	int _border;
};


/**
 * The UPlot legend item. Used internally by UPlot.
 */
class UPlotLegendItem : public QPushButton
{
	Q_OBJECT

public:
	/**
	 * Constructor.
	 */
	UPlotLegendItem(const UPlotCurve * curve, QWidget * parent = 0);
	virtual ~UPlotLegendItem();
	const UPlotCurve * curve() const {return _curve;}

Q_SIGNALS:
	void legendItemRemoved(const UPlotCurve *);

protected:
	virtual void contextMenuEvent(QContextMenuEvent * event);

private:
	const UPlotCurve * _curve;
	QMenu * _menu;
	QAction * _aChangeText;
	QAction * _aResetText;
	QAction * _aRemoveCurve;
	QAction * _aCopyToClipboard;
};

/**
 * The UPlot legend. Used internally by UPlot.
 */
class UPlotLegend : public QWidget
{
	Q_OBJECT

public:
	/**
	 * Constructor.
	 */
	UPlotLegend(QWidget * parent = 0);
	virtual ~UPlotLegend();

	void setFlat(bool on);
	bool isFlat() const {return _flat;}
	void addItem(const UPlotCurve * curve);
	QPixmap createSymbol(const QPen & pen, const QBrush & brush);
	bool remove(const UPlotCurve * curve);

public Q_SLOTS:
	void removeLegendItem(const UPlotCurve * curve);

Q_SIGNALS:
	void legendItemRemoved(const UPlotCurve * curve);
	void legendItemToggled(const UPlotCurve * curve, bool toggled);

protected:
	virtual void contextMenuEvent(QContextMenuEvent * event);

private Q_SLOTS:
	void redirectToggled(bool);

private:
	bool _flat;
	QMenu * _menu;
	QAction * _aUseFlatButtons;
};


/**
 * Orientable QLabel. Inherit QLabel and let you to specify the orientation.
 */
class UOrientableLabel : public QLabel
{
	Q_OBJECT

public:
	/**
	 * Constructor.
	 */
	UOrientableLabel(const QString & text, Qt::Orientation orientation = Qt::Horizontal, QWidget * parent = 0);
	virtual ~UOrientableLabel();
	/**
	 * Get orientation.
	 */
	Qt::Orientation orientation() const {return _orientation;}
	/**
	 * Set orientation (Qt::Vertical or Qt::Horizontal).
	 */
	void setOrientation(Qt::Orientation orientation);
	QSize sizeHint() const;
	QSize minimumSizeHint() const;
protected:
    virtual void paintEvent(QPaintEvent* event);
private:
    Qt::Orientation _orientation;
};

/**
 * UPlot is a QWidget to create a plot like MATLAB, and
 * incrementally add new values like a scope using Qt signals/slots.
 * Many customizations can be done at runtime with the right-click menu.
 * @image html UPlot.gif
 * @image html UPlotMenu.png
 *
 * Example:
 * @code
 * #include "utilite/UPlot.h"
 * #include <QApplication>
 *
 * int main(int argc, char * argv[])
 * {
 *	QApplication app(argc, argv);
 *	UPlot plot;
 *	UPlotCurve * curve = plot.addCurve("My curve");
 *	float y[10] = {0, 1, 2, 3, -3, -2, -1, 0, 1, 2};
 *	curve->addValues(std::vector<float>(y, y+10));
 *	plot.showGrid(true);
 *	plot.setGraphicsView(true);
 *	plot.show();
 *	app.exec();
 * 	return 0;
 * }
 * @endcode
 * @image html SimplePlot.tiff
 *
 *
 */
class UPlot : public QWidget
{
	Q_OBJECT

public:
	/**
	 * Constructor.
	 */
	UPlot(QWidget * parent = 0);
	virtual ~UPlot();

	/**
	 * Add a curve. The returned curve doesn't need to be deallocated (UPlot keeps the ownership).
	 */
	UPlotCurve * addCurve(const QString & curveName, const QColor & color = QColor());
	/**
	 * Add a curve. Ownership is transferred to UPlot if ownershipTransferred=true.
	 */
	bool addCurve(UPlotCurve * curve, bool ownershipTransferred = true);
	/**
	 * Get all curve names.
	 */
	QStringList curveNames();
	bool contains(const QString & curveName);
	void removeCurves();
	/**
	 * Add a threshold to the plot.
	 */
	UPlotCurveThreshold * addThreshold(const QString & name, float value, Qt::Orientation orientation = Qt::Horizontal);
	QString title() const {return this->objectName();}
	QPen getRandomPenColored();
	void showLegend(bool shown);
	void showGrid(bool shown);
	void showRefreshRate(bool shown);
	void keepAllData(bool kept);
	void showXAxis(bool shown) {_horizontalAxis->setVisible(shown);}
	void showYAxis(bool shown) {_verticalAxis->setVisible(shown);}
	void setVariableXAxis() {_fixedAxis[0] = false;}
	void setVariableYAxis() {_fixedAxis[1] = false;}
	void setFixedXAxis(float x1, float x2);
	void setFixedYAxis(float y1, float y2);
	void setMaxVisibleItems(int maxVisibleItems);
	void setTitle(const QString & text);
	void setXLabel(const QString & text);
	void setYLabel(const QString & text, Qt::Orientation orientation = Qt::Vertical);
	void setWorkingDirectory(const QString & workingDirectory);
	void setGraphicsView(bool on);
	QRectF sceneRect() const;

public Q_SLOTS:
	/**
	 *
	 * Remove a curve. If UPlot is the parent of the curve, the curve is deleted.
	 */
	void removeCurve(const UPlotCurve * curve);
	void showCurve(const UPlotCurve * curve, bool shown);
	void updateAxis(); //reset axis and recompute it with all curves minMax
	/**
	 *
	 * Clear all curves' data.
	 */
	void clearData();

private Q_SLOTS:
	void captureScreen();
	void updateAxis(const UPlotCurve * curve);

protected:
	virtual void contextMenuEvent(QContextMenuEvent * event);
	virtual void paintEvent(QPaintEvent * event);
	virtual void resizeEvent(QResizeEvent * event);

private:
	friend class UPlotCurve;
	void addItem(QGraphicsItem * item);

private:
	void replot(QPainter * painter);
	bool updateAxis(float x, float y);
	bool updateAxis(float x1, float x2, float y1, float y2);
	void setupUi();
	void createActions();
	void createMenus();
	void selectScreenCaptureFormat();

private:
	UPlotLegend * _legend;
	QGraphicsView * _view;
	QGraphicsItem * _sceneRoot;
	QWidget * _graphicsViewHolder;
	float _axisMaximums[4]; // {x1->x2, y1->y2}
	bool _axisMaximumsSet[4]; // {x1->x2, y1->y2}
	bool _fixedAxis[2];
	UPlotAxis * _verticalAxis;
	UPlotAxis * _horizontalAxis;
	int _penStyleCount;
	int _maxVisibleItems;
	QList<QGraphicsLineItem *> hGridLines;
	QList<QGraphicsLineItem *> vGridLines;
	QList<UPlotCurve*> _curves;
	QLabel * _title;
	QLabel * _xLabel;
	UOrientableLabel * _yLabel;
	QLabel * _refreshRate;
	QString _workingDirectory;
	QElapsedTimer _refreshIntervalTime;
	int _lowestRefreshRate;
	QElapsedTimer _refreshStartTime;
	QString _autoScreenCaptureFormat;

	QMenu * _menu;
	QAction * _aShowLegend;
	QAction * _aShowGrid;
	QAction * _aKeepAllData;
	QAction * _aLimit0;
	QAction * _aLimit10;
	QAction * _aLimit50;
	QAction * _aLimit100;
	QAction * _aLimit500;
	QAction * _aLimit1000;
	QAction * _aLimitCustom;
	QAction * _aAddVerticalLine;
	QAction * _aAddHorizontalLine;
	QAction * _aChangeTitle;
	QAction * _aChangeXLabel;
	QAction * _aChangeYLabel;
	QAction * _aYLabelVertical;
	QAction * _aShowRefreshRate;
	QAction * _aSaveFigure;
	QAction * _aAutoScreenCapture;
	QAction * _aClearData;
	QAction * _aGraphicsView;
};

#endif /* UPLOT_H_ */
