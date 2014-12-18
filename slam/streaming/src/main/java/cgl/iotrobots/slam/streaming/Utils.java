package cgl.iotrobots.slam.streaming;

import cgl.iotrobots.slam.core.grid.GMap;
import cgl.iotrobots.slam.core.scanmatcher.PointAccumulator;
import cgl.iotrobots.slam.core.utils.IntPoint;
import cgl.iotrobots.slam.streaming.msgs.MapCell;
import cgl.iotrobots.slam.streaming.msgs.TransferMap;
import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.io.Input;
import com.esotericsoftware.kryo.io.Output;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;

/**
 * Utilities
 */
public class Utils {
    /**
     * Serialize an object using kryo and return the bytes
     * @param kryo instance of kryo
     * @param object the object to be serialized
     * @return the serialized bytes
     */
    public static byte[] serialize(Kryo kryo, Object object) {
        ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream();
        Output output = new Output(byteArrayOutputStream);
//        output.setOutputStream(byteArrayOutputStream);
        kryo.writeObject(output, object);
        output.flush();
        return byteArrayOutputStream.toByteArray();
    }

    /**
     * De Serialize bytes using kryo and return the object
     * @param kryo instance of kryo
     * @param b the byte to be de serialized
     * @return the serialized bytes
     */
    public static Object deSerialize(Kryo kryo, byte []b, Class e) {
        return kryo.readObject(new Input(new ByteArrayInputStream(b)), e);
    }

    public static TransferMap createTransferMap(GMap map) {
        TransferMap transferMap = new TransferMap(map.getCenter(), map.getWorldSizeX(),
                map.getWorldSizeY(), map.getDelta(), map.getMapSizeX(),
                map.getMapSizeY(), map.getSizeX2(), map.getSizeY2(),
                map.getStorage().getPatchMagnitude(),
                map.getStorage().getPatchSize());

        for (int x = 0; x < map.getMapSizeX(); x++) {
            for (int y = 0; y < map.getMapSizeY(); y++) {
                /// @todo Sort out the unknown vs. free vs. obstacle thresholding
                IntPoint p = new IntPoint(x, y);
                PointAccumulator pointAccumulator = (PointAccumulator) map.cell(p, false);
                MapCell cell = createMapCell(pointAccumulator, x, y);
                transferMap.addCell(cell);
            }
        }

//        transferMap.setActiveArea(map.getStorage().getActiveArea());
        return transferMap;
    }

    public static GMap createGMap(TransferMap tMap) {
        GMap gMap = new GMap(tMap.getCenter(), tMap.getWorldSizeX(), tMap.getWorldSizeY(), tMap.getDelta());

        for (MapCell cell : tMap.getMapCells()) {
            PointAccumulator accumulator = (PointAccumulator) gMap.cell(cell.getX(), cell.getY(), false);
            accumulator.setAcc(cell.getAcc());
            accumulator.setN(cell.getN());
            accumulator.setVisits(cell.getVisits());
        }

//        if (tMap.getActiveArea() != null) {
//            gMap.getStorage().setActiveArea(tMap.getActiveArea());
//        }

        return gMap;
    }

    public static MapCell createMapCell(PointAccumulator acc, int x, int y) {
        return new MapCell(x, y, acc.getAcc(), acc.getN(), acc.getVisits());
    }
}
