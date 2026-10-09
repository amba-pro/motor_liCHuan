#include "live_telemetry.hpp"
#include <QtTest>
#include <QLocalServer>
#include <QLocalSocket>
#include <QUuid>

class LiveTelemetryTest : public QObject {
  Q_OBJECT
 private slots:
  void parsesRealProtocolOnly();
  void refusesMalformedAndStaleSamples();
};

void LiveTelemetryTest::parsesRealProtocolOnly() {
  QLocalServer server;
  const QString name = "lichuan-test-" + QUuid::createUuid().toString(QUuid::WithoutBraces);
  QVERIFY(server.listen(name));
  LiveTelemetry reader;
  reader.connectToLocalService(name);
  QVERIFY(server.waitForNewConnection(2000));
  QScopedPointer<QLocalSocket> peer(server.nextPendingConnection());
  QVERIFY(peer);
  const QByteArray data = R"({"schema":1,"source":"lc_e_csp_hold","position_counts":-4224994,"velocity_counts_s":0,"torque_raw":0,"following_counts":0,"statusword":545,"error_code":0,"wkc":3,"op":true,"enabled":false})" "\n";
  peer->write(data);
  peer->flush();
  QTRY_VERIFY(reader.fresh());
  QCOMPARE(reader.snapshot().positionCounts, qint64(-4224994));
  QCOMPARE(reader.snapshot().wkc, 3);
  QVERIFY(!reader.snapshot().enabled);
}

void LiveTelemetryTest::refusesMalformedAndStaleSamples() {
  QLocalServer server;
  const QString name = "lichuan-test-" + QUuid::createUuid().toString(QUuid::WithoutBraces);
  QVERIFY(server.listen(name));
  LiveTelemetry reader;
  reader.connectToLocalService(name);
  QVERIFY(server.waitForNewConnection(2000));
  QScopedPointer<QLocalSocket> peer(server.nextPendingConnection());
  peer->write("{\"schema\":1,\"source\":\"demo\"}\n");
  peer->flush();
  QTest::qWait(50);
  QVERIFY(!reader.fresh());
  const QByteArray valid = R"({"schema":1,"source":"lc_e_csp_hold","position_counts":1,"velocity_counts_s":0,"torque_raw":0,"following_counts":0,"statusword":545,"error_code":0,"wkc":3,"op":true,"enabled":false})" "\n";
  peer->write(valid);
  peer->flush();
  QTRY_VERIFY(reader.fresh());
  QTest::qWait(550);
  QVERIFY(!reader.fresh());
}
QTEST_GUILESS_MAIN(LiveTelemetryTest)
#include "test_live_telemetry.moc"
