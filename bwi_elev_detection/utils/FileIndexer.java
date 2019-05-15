import java.io.File;
import java.io.IOException;
import java.io.PrintStream;
import java.util.Collections;
import java.util.HashSet;

/**
 * Created by brian on 4/17/14.
 */
public class FileIndexer {
    static PrintStream out;
    static HashSet<String> acceptedExtensions;

    public static void main(String... args) throws IOException {
        out = new PrintStream(args.length >= 2 ? args[1] : "FileIndex.txt");
        File file = new File(args.length >= 1 ? args[0] : ".");

        String[] ext = {"jpg", "png", "jpeg", "bmp"};

        acceptedExtensions = new HashSet<String>();
        Collections.addAll(acceptedExtensions, ext);

        explore(file);
        System.out.println("Indexing finished");
    }

    public static void explore(File root) throws IOException {
        if (root == null || root.listFiles() == null)
            return;

        System.out.println("Indexing directory " + root.getPath());
        for (File f : root.listFiles()) {
            if (f == null)
                continue;
            if (f.isDirectory())
                explore(f);
            else if (acceptedExtensions.contains(getExtension(f.getName()))){
                // System.out.println(f.getPath());
                out.println(f.getPath().substring(f.getPath().indexOf("/") + 1));
            }
        }
    }

    private static String getExtension(String name) {
        return name.substring(name.lastIndexOf(".") + 1).toLowerCase();
    }
}