// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import java.util.Comparator;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/**
 * Interpolating Tree Maps are used to get values at points that are not defined by making a guess
 * from points that are defined. This uses linear interpolation.
 *
 * Extended from WPILib's InterpolatingTreeMap to expose entrySet() and lastKey().
 *
 * @param <K>
 *            The type of keys held in this map.
 * @param <V>
 *            The type of values held in this map.
 */
public class WildcardInterpolatingTreeMap<K, V>
{
    private final TreeMap<K, V> m_map;
    private final InverseInterpolator<K> m_inverseInterpolator;
    private final Interpolator<V> m_interpolator;

    public WildcardInterpolatingTreeMap(InverseInterpolator<K> inverseInterpolator, Interpolator<V> interpolator)
    {
        m_map = new TreeMap<>();
        m_inverseInterpolator = inverseInterpolator;
        m_interpolator = interpolator;
    }

    public WildcardInterpolatingTreeMap(
            InverseInterpolator<K> inverseInterpolator,
            Interpolator<V> interpolator,
            Comparator<K> comparator)
    {
        m_inverseInterpolator = inverseInterpolator;
        m_interpolator = interpolator;
        m_map = new TreeMap<>(comparator);
    }

    public void put(K key, V value)
    {
        m_map.put(key, value);
    }

    public V get(K key)
    {
        V val = m_map.get(key);
        if (val == null)
        {
            K ceilingKey = m_map.ceilingKey(key);
            K floorKey = m_map.floorKey(key);

            if (ceilingKey == null && floorKey == null)
            {
                return null;
            }
            if (ceilingKey == null)
            {
                return m_map.get(floorKey);
            }
            if (floorKey == null)
            {
                return m_map.get(ceilingKey);
            }
            V floor = m_map.get(floorKey);
            V ceiling = m_map.get(ceilingKey);

            return m_interpolator.interpolate(
                    floor, ceiling, m_inverseInterpolator.inverseInterpolate(floorKey, ceilingKey, key));
        }
        else
        {
            return val;
        }
    }

    public void clear()
    {
        m_map.clear();
    }

    public Set<Map.Entry<K, V>> entrySet()
    {
        return m_map.entrySet();
    }

    public K lastKey()
    {
        return m_map.lastKey();
    }
}
