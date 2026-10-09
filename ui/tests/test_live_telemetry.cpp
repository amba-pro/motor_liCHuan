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
  void acceptsExplicitlyUnavailableVelocity();
  void reconnectsAfterProducerRestart();
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

void LiveTelemetryTest::acceptsExplicitlyUnavailableVelocity() {
  QLocalServer server;
  const QString name = "lichuan-test-" + QUuid::createUuid().toString(QUuid::WithoutBraces);
  QVERIFY(server.listen(name));
  LiveTelemetry reader;
  reader.connectToLocalService(name);
  QVERIFY(server.waitForNewConnection(2000));
  QScopedPointer<QLocalSocket> peer(server.nextPendingConnection());
  const QByteArray data =
      R"({"schema":1,"source":"lc_e_csp_hold","position_counts":-100,"velocity_counts_s":null,"torque_raw":0,"following_counts":5,"statusword":592,"error_code":0,"wkc":3,"op":true,"enabled":false})"
      "\n";
  peer->write(data);
  peer->flush();
  QTRY_VERIFY(reader.fresh());
  QVERIFY(!reader.snapshot().velocityKnown);
  QCOMPARE(reader.snapshot().followingCounts, 5);
}

void LiveTelemetryTest::reconnectsAfterProducerRestart() {
  QLocalServer server;
  const QString name = "lichuan-test-" + QUuid::createUuid().toString(QUuid::WithoutBraces);
  QVERIFY(server.listen(name));
  LiveTelemetry reader;
  reader.connectToLocalService(name);
  QVERIFY(server.waitForNewConnection(2000));
  QScopedPointer<QLocalSocket> peer(server.nextPendingConnection());
  const QByteArray first =
      R"({"schema":1,"source":"lc_e_csp_hold","position_counts":1,"velocity_counts_s":null,"torque_raw":0,"following_counts":0,"statusword":592,"error_code":0,"wkc":3,"op":true,"enabled":false})"
      "\n";
  peer->write(first);
  peer->flush();
  QTRY_VERIFY(reader.fresh());
  peer->disconnectFromServer();
  server.close();
  QLocalServer::removeServer(name);
  QTRY_VERIFY(!reader.connected());
  QVERIFY(!reader.fresh());
  QVERIFY(server.listen(name));
  QTRY_VERIFY_WITH_TIMEOUT(server.hasPendingConnections(), 2000);
  QScopedPointer<QLocalSocket> again(server.nextPendingConnection());
  QVERIFY(again);
  const QByteArray second =
      R"({"schema":1,"source":"lc_e_csp_hold","position_counts":2,"velocity_counts_s":null,"torque_raw":0,"following_counts":0,"statusword":592,"error_code":0,"wkc":3,"op":true,"enabled":false})"
      "\n";
  again->write(second);
  again->flush();
  QTRY_VERIFY_WITH_TIMEOUT(reader.fresh() && reader.snapshot().positionCounts == 2, 2000);
}
QTEST_GUILESS_MAIN(LiveTelemetryTest)
#include "test_live_telemetry.moc"
