#include <QtGui/QApplication>
#include "MainWindow.h"

int main(int argc, char* argv[])
{
	/* Create tasks */
	QApplication app(argc, argv);
	MainWindow mainWindow;

	mainWindow.showNormal();

	// Now wait for application to finish
	app.connect( &app, SIGNAL( lastWindowClosed() ),
				&app, SLOT( quit() ) );
	app.exec();// MUST be called by the Main Thread

    return 0;
}
